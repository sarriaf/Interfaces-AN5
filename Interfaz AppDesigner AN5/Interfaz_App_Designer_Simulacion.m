classdef Interfaz_App_Designer_Simulacion < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        TabGroup                  matlab.ui.container.TabGroup
        SimulinkTab               matlab.ui.container.Tab
        SIMULARLabel              matlab.ui.control.Label
        RespuestaTextArea         matlab.ui.control.TextArea
        RespuestaTextAreaLabel    matlab.ui.control.Label
        RESETButton               matlab.ui.control.Button
        Button_4                  matlab.ui.control.Button
        ESPACIODESIMULACIONLabel  matlab.ui.control.Label
        DETENERButton_3           matlab.ui.control.Button
        MOVERButton_3             matlab.ui.control.Button
        SimulationControls        simulink.ui.control.SimulationControls
        CARGARTXTButton           matlab.ui.control.Button
    end


    % Public properties that correspond to the Simulink model
    properties (Access = public, Transient)
        Simulation simulink.Simulation
    end


  properties (Access = public)
        ExternalAxes          matlab.graphics.axis.Axes
        
    end
    
   properties (Access = private)
        robot                         % Modelo del robot (rigidBodyTree)
        node                          % Nodo ROS2 para la comunicación
        pub_input_cartesian_position  % Publicador para 'input_cartesian_position'
        sub_output_joint_position     % Suscriptor para 'output_joint_position'
        sub_current_joint_position    % Suscriptor para 'current_joint_position'
        pub_api_command               % Publicador para 'api_command'
        pub_input_cartesian_path      % Publicador para 'input_cartesian_path'
        initialguess                  % Configuración inicial del robot
        ax                            % Ejes para la visualización
        isArticularMode = false       % Indicador de modo articular activo
        fig                           % Figura para la visualización 3D
        lastJointPositions            % Almacenar las últimas posiciones conocidas de las articulaciones
        updateThreshold = 0.2;        % Umbral de actualización en grados
        % Propiedades para almacenar posiciones y rotaciones
        positions = [];               % Matriz para almacenar las posiciones cartesianas
        rotations = [];               % Matriz para almacenar las rotaciones (roll, pitch, yaw)
        num_points = 0;               % Número de puntos añadidos
        configs_matrix = [];          % Matriz para almacenar las configuraciones articulares calculadas
        posicionesTxtFile             % Propiedad para almacenar la ruta del archivo .txt cargado
        generatedFigures = [];        % Almacena las figuras creadas por la interfaz
   end

   methods (Access = public)
    % Método para mostrar mensajes en el RespuestaTextArea
    function logInTextArea(app, txt)
        currentText = app.RespuestaTextArea.Value;
        if ischar(currentText)
            currentText = {currentText};
        end
        newLine = sprintf('%s', txt);
        app.RespuestaTextArea.Value = [currentText; {newLine}];
        drawnow;
    end
end


 methods (Access = private)
        %% Función para iniciar la simulación y vincular el progreso
        function startSimulation(app)
            % Configura el objeto de simulación y vincula a SimulationProgress
            app.Simulation = simulacion('simulador_trayectorias_AN5'); % Reemplaza con el nombre exacto
            app.SimulationProgress.Simulation = app.Simulation; % Vincula SimulationProgress

            % Actualiza el tiempo de simulación en TimeLabel
            addlistener(app.SimulationProgress, 'ProgressChanged', @(~,~) app.updateSimulationTime());

            % Inicia la simulación
            start(app.Simulation); % Asegúrate de que tu simulación inicie correctamente
        end

        %% Función para actualizar el tiempo de simulación en el Label
        function updateSimulationTime(app)
            % Obtén el tiempo de simulación actual
            simTime = app.SimulationProgress.Simulation.getSimulationTime();
            app.TimeLabel.Text = sprintf('Tiempo de simulación: %.2f s', simTime);
        end

        %% Función para cargar y visualizar el URDF del robot
        function loadAndVisualizeURDF(app)
            archivoURDF = 'fr5v6.urdf'; % Nombre del archivo URDF del robot

            % Importa el modelo del robot si aún no está cargado
            if isempty(app.robot)
                if isfile(archivoURDF)
                    app.robot = importrobot(archivoURDF);
                else
                    error('Archivo URDF no encontrado. Verifica la ruta.');
                end
            end

            % Crea la figura y los ejes para la visualización si no existen
            if isempty(app.fig) || ~isvalid(app.fig)
                screenSize = get(0, 'ScreenSize');
                screenWidth = screenSize(3);
                screenHeight = screenSize(4);

                app.fig = figure('Name', 'Visualización AN5 en Tiempo Real', ...
                                 'Position', [screenWidth / 2, 1, screenWidth / 2, screenHeight], ...
                                 'CloseRequestFcn', @(src, event)app.onCloseVisualizationFigure());
                app.ax = axes(app.fig);
            end

            % Muestra el modelo del robot en los ejes definidos
            if ~isempty(app.ax) && isvalid(app.ax)
                show(app.robot, 'Parent', app.ax);
                axis(app.ax, 'vis3d');

                % Configura la vista y los límites de los ejes para una mejor visualización
                view(app.ax, [18 20]);
                app.ax.XLim = [-1 1];
                app.ax.YLim = [-1 1];
                app.ax.ZLim = [0 1.2];
                hold(app.ax, 'on');
                hold(app.ax, 'off');
                rotate3d(app.ax, 'on'); % Habilita la rotación interactiva
                app.initialguess = app.robot.homeConfiguration; % Configuración inicial
                disp('Modelo URDF cargado y visualizado con ajustes de vista.');
            end
        end

        %% Función para manejar el cierre de la figura de visualización
        function onCloseVisualizationFigure(app)
            if ~isempty(app.fig) && isvalid(app.fig)
                try
                    % Detiene cualquier suscripción activa antes de cerrar
                    app.stopJointPositionSubscriber();

                    delete(app.fig);
                    app.fig = [];
                    app.ax = [];
                    disp('Figura de visualización cerrada.');
                catch ME
                    disp(['Error al cerrar la figura: ', ME.message]);
                end
            else
                disp('La figura ya está cerrada o no es válida.');
            end
        end

        %% Función para cerrar solo las figuras generadas por la interfaz
        function closeGeneratedFigures(app)
            if ~isempty(app.generatedFigures)
                for i = 1:length(app.generatedFigures)
                    if isvalid(app.generatedFigures(i))
                        delete(app.generatedFigures(i));
                    end
                end
                app.generatedFigures = [];
                disp('Todas las figuras generadas por la interfaz han sido cerradas.');
            end
        end
 


        %% Función para actualizar el robot con una sola articulación
        function updateRobotSingleJoint(app, jointIndex, jointAngle)
            configSol = app.initialguess;
            configSol(jointIndex).JointPosition = deg2rad(jointAngle);

            % Recupera los ángulos actuales de las articulaciones desde los campos de edición
            jointAngles = [
                app.EditFieldJoint1.Value, ...
                app.EditFieldJoint2.Value, ...
                app.EditFieldJoint3.Value, ...
                app.EditFieldJoint4.Value, ...
                app.EditFieldJoint5.Value, ...
                app.EditFieldJoint6.Value];

            for i = 1:length(jointAngles)
                if i ~= jointIndex
                    configSol(i).JointPosition = deg2rad(jointAngles(i));
                end
            end

            cla(app.ax);
            show(app.robot, configSol, 'Parent', app.ax);
            axis(app.ax, 'vis3d');
            view(app.ax, [18 20]);
            app.ax.XLim = [-1 1];
            app.ax.YLim = [-1 1];
            app.ax.ZLim = [0 1.2];
            hold(app.ax, 'off');
            rotate3d(app.ax, 'on');
        end

        %% Función para actualizar el robot desde los sliders
        function updateRobotFromSliders(app)
            if ~isempty(app.ax) && isvalid(app.ax)
                cla(app.ax);

                configSol = app.initialguess;
                jointAngles = [
                    app.SliderJoint1.Value, ...
                    app.SliderJoint2.Value, ...
                    app.SliderJoint3.Value, ...
                    app.SliderJoint4.Value, ...
                    app.SliderJoint5.Value, ...
                    app.SliderJoint6.Value];

                for i = 1:length(jointAngles)
                    configSol(i).JointPosition = deg2rad(jointAngles(i));
                end

                show(app.robot, configSol, 'Parent', app.ax);
                axis(app.ax, 'vis3d');
                view(app.ax, [18 20]);
                app.ax.XLim = [-1 1];
                app.ax.YLim = [-1 1];
                app.ax.ZLim = [0 1.2];
                hold(app.ax, 'off');
                rotate3d(app.ax, 'on');
            else
                disp('Advertencia: No hay un eje válido para la visualización.');
            end
        end

        %% Función para actualizar el robot desde los ángulos articulares
        function updateRobotFromJointAngles(app, jointAngles)
            if ~isempty(app.ax) && isvalid(app.ax)
                cla(app.ax);

                configSol = app.initialguess;
                for i = 1:length(jointAngles)
                    configSol(i).JointPosition = deg2rad(jointAngles(i));
                end

                show(app.robot, configSol, 'Parent', app.ax);
                axis(app.ax, 'vis3d');
                view(app.ax, [18 20]);
                app.ax.XLim = [-1 1];
                app.ax.YLim = [-1 1];
                app.ax.ZLim = [0 1.2];
                hold(app.ax, 'off');
                rotate3d(app.ax, 'on');
            else
                disp('Advertencia: No hay un eje válido para la visualización.');
            end
        end

        %% Función para iniciar el suscriptor de posiciones de articulaciones
        function startJointPositionSubscriber(app)
            if isempty(app.node)
                app.node = ros2node('matlab_sub_node');
            end

            if isempty(app.sub_current_joint_position)
                app.sub_current_joint_position = ros2subscriber(app.node, 'current_joint_position', 'std_msgs/String', @app.jointPositionCallback);
                disp('Suscripción a posiciones de articulaciones iniciada.');
            end
        end

        %% Función para detener el suscriptor de posiciones de articulaciones
        function stopJointPositionSubscriber(app)
            if ~isempty(app.sub_current_joint_position) && isvalid(app.sub_current_joint_position)
                delete(app.sub_current_joint_position);
                app.sub_current_joint_position = [];
                disp('Suscripción a posiciones de articulaciones detenida.');
            else
                disp('No hay suscripción activa para detener.');
            end
        end

        %% Callback para recibir las posiciones de articulaciones
        function jointPositionCallback(app, msg)
            jointPositions = str2double(strsplit(msg.data, ','));

            if length(jointPositions) == 6 && ~app.isArticularMode
                app.SliderJoint1.Value = jointPositions(1);
                app.EditFieldJoint1.Value = jointPositions(1);

                app.SliderJoint2.Value = jointPositions(2);
                app.EditFieldJoint2.Value = jointPositions(2);

                app.SliderJoint3.Value = jointPositions(3);
                app.EditFieldJoint3.Value = jointPositions(3);

                app.SliderJoint4.Value = jointPositions(4);
                app.EditFieldJoint4.Value = jointPositions(4);

                app.SliderJoint5.Value = jointPositions(5);
                app.EditFieldJoint5.Value = jointPositions(5);

                app.SliderJoint6.Value = jointPositions(6);
                app.EditFieldJoint6.Value = jointPositions(6);

                app.updateRobotFromSliders();
            end
        end

        %% Callback para recibir la solución de la cinemática inversa
        function inversaOutputCallback(app, msg)
            try
                jointAngles = str2double(strsplit(msg.data, ','));

                if length(jointAngles) == 6 && all(~isnan(jointAngles))
                    app.EditFieldJoint1.Value = jointAngles(1);
                    app.EditFieldJoint2.Value = jointAngles(2);
                    app.EditFieldJoint3.Value = jointAngles(3);
                    app.EditFieldJoint4.Value = jointAngles(4);
                    app.EditFieldJoint5.Value = jointAngles(5);
                    app.EditFieldJoint6.Value = jointAngles(6);

                    app.SliderJoint1.Value = jointAngles(1);
                    app.SliderJoint2.Value = jointAngles(2);
                    app.SliderJoint3.Value = jointAngles(3);
                    app.SliderJoint4.Value = jointAngles(4);
                    app.SliderJoint5.Value = jointAngles(5);
                    app.SliderJoint6.Value = jointAngles(6);

                    app.updateRobotFromJointAngles(jointAngles);
                    disp('Articulaciones actualizadas con la solución de cinemática inversa.');
                else
                    disp('Error: Se esperaban 6 valores válidos para la cinemática inversa.');
                end
            catch ME
                disp(['Error al procesar la solución de cinemática inversa: ', ME.message]);
            end
        end

        %% Función para inicializar el switch y los LEDs
        function initializeSwitchAndLeds(app)
            app.AutoManualSwitch.Value = 'Automatico';
            app.AutoManualSwitch_2.Value = 'Automatico';
            app.sendDragTeachSwitch(0);
            app.Led_manu.Color = [0 1 0]; 
            app.Led_Auto.Color = [1 1 1];  
            app.Led_Manual.Color = [0 1 0];  
            app.Led_Auto.Color = [1 1 1];  
        end

        %% Función para enviar el comando DragTeachSwitch
        function sendDragTeachSwitch(app, mode)
            if isempty(app.node)
                app.node = ros2node('matlab_pub_node');
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            msg = ros2message(app.pub_api_command);

            if mode == 0
                msg.data = sprintf('DragTeachSwitch(%d)', mode);
                send(app.pub_api_command, msg);

                msg.data = 'SplineEnd()';
                send(app.pub_api_command, msg);

                msg.data = 'ResetAllError()';
                send(app.pub_api_command, msg);

                msg.data = 'StartJOG(0,6,0,100)';
                send(app.pub_api_command, msg);

                msg.data = 'StartJOG(1,6,0,100)';
                send(app.pub_api_command, msg);

                disp(['Publicado: ', msg.data]);
            end

            if mode == 1
                msg.data = sprintf('DragTeachSwitch(%d)', mode);
                send(app.pub_api_command, msg);
                disp(['Publicado: ', msg.data]);
            end
        end

        %% Función para enviar la posición cartesiana
        function sendCartesianPosition(app)
            x = app.XEditField.Value;
            y = app.YEditField.Value;
            z = app.ZEditField.Value;
            rx = app.RxEditField.Value;
            ry = app.RyEditField.Value;
            rz = app.RzEditField.Value;

            input_str = sprintf('%g,%g,%g,%g,%g,%g', x, y, z, rx, ry, rz);

            if isempty(app.pub_input_cartesian_position)
                app.pub_input_cartesian_position = ros2publisher(app.node, 'input_cartesian_position', 'std_msgs/String');
            end
            msg = ros2message(app.pub_input_cartesian_position);
            msg.data = input_str;
            send(app.pub_input_cartesian_position, msg);
            disp(['Posición enviada para cinemática inversa: ', input_str]);
        end

        %% Función para inicializar ROS
        function initializeROS(app)
            if isempty(app.node)
                app.node = ros2node('matlab_node2');
            end

            if isempty(app.pub_input_cartesian_path)
                app.pub_input_cartesian_path = ros2publisher(app.node, 'input_cartesian_path', 'std_msgs/String');
            end

            if isempty(app.sub_output_joint_position)
                app.sub_output_joint_position = ros2subscriber(app.node, 'output_joint_position', 'std_msgs/String', @app.inversaOutputCallback);
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end
        end
        
 
    

    end

    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: CARGARTXTButton
function CARGARTXTButtonPushed(app, event)
    [file, path] = uigetfile('*.txt', 'Seleccione el archivo de posiciones');
    if isequal(file, 0)
        app.logInTextArea('No se seleccionó ningún archivo.');
        return;
    end

    % Cierra todas las figuras generadas previamente
    app.closeGeneratedFigures();
    app.generatedFigures = [];
    close all;

    % Ruta completa del archivo seleccionado
    archivo = fullfile(path, file);
    app.logInTextArea(['Archivo seleccionado: ', archivo]);

    % Sobrescribir el archivo como desired_positions.txt
    archivo_destino = 'desired_positions.txt';
    copyfile(archivo, archivo_destino);
    app.logInTextArea(['Archivo cargado y guardado como: ', archivo_destino]);

    % Leer el archivo ignorando la primera fila (cabecera) y calcular Tfinal
    data = dlmread(archivo_destino, ',', 1, 0);
    
    num_puntos = size(data, 1);
    Tfinal = num_puntos * 0.1;
    assignin('base', 'Tfinal', Tfinal);
    app.logInTextArea(['Número de puntos encontrados: ', num2str(num_puntos)]);
    app.logInTextArea(['Tfinal calculado: ', num2str(Tfinal), ' segundos']);
end

        % Button pushed function: MOVERButton_3
        function MOVERButton_3Pushed(app, event)
% Nombre del archivo que siempre se enviará
            archivo_destino = 'desired_positions.txt';

            % Inicializar nodo y publicador ROS si no existen
            if isempty(app.node)
                app.node = ros2node('matlab_pub_node'); % Nombre del nodo ROS
            end

            if isempty(app.pub_input_cartesian_path)
                app.pub_input_cartesian_path = ros2publisher(app.node, 'input_cartesian_path', 'std_msgs/String');
            end

            % Crear mensaje ROS
            msg = ros2message(app.pub_input_cartesian_path);
            msg.data = archivo_destino;  % Asigna el nombre del archivo al mensaje

            % Publicar el mensaje en el tópico
            send(app.pub_input_cartesian_path, msg);
            disp(['Mensaje enviado con el archivo: ', archivo_destino]);
            disp(['Mensaje enviado con el archivo: ', archivo_destino]);
        end

        % Button pushed function: DETENERButton_3
        function DETENERButton_3Pushed(app, event)
 % Lista de comandos a enviar en orden
            commands = {
                'SplineEnd()', 
                'StopMotion()', 
                'ResetAllError()', 
                'StartJOG(0,6,0,100)', 
                'StartJOG(1,6,0,100)'
            };

            % Inicializar nodo y publicador ROS si no existen
            if isempty(app.node)
                app.node = ros2node('matlab_pub_node'); % Nombre del nodo ROS
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            % Enviar los comandos en orden
            for i = 1:length(commands)
                cmd = commands{i};

                % Crear mensaje y asignar el comando
                msg = ros2message(app.pub_api_command);
                msg.data = cmd;

                % Publicar el mensaje
                send(app.pub_api_command, msg);
                disp(['Publicado: ', msg.data]);

                pause(0.05);  % Pequeño retraso entre comandos
            end

        end

        % Button pushed function: RESETButton
        function RESETButtonPushed(app, event)
             % Lista de comandos a enviar en orden
            commands = {
                'SplineEnd()', 
                'StopMotion()', 
                'ResetAllError()', 
                'StartJOG(0,6,0,100)', 
                'StartJOG(1,6,0,100)'
            };

            % Inicializar nodo y publicador ROS si no existen
            if isempty(app.node)
                app.node = ros2node('matlab_pub_node'); % Nombre del nodo ROS
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            % Enviar los comandos en orden
            for i = 1:length(commands)
                cmd = commands{i};

                % Crear mensaje y asignar el comando
                msg = ros2message(app.pub_api_command);
                msg.data = cmd;

                % Publicar el mensaje
                send(app.pub_api_command, msg);
                disp(['Publicado: ', msg.data]);

                pause(0.05);  % Pequeño retraso entre comandos
            end

        end

        % Value changed function: RespuestaTextArea
        function RespuestaTextAreaValueChanged(app, event)

    max_length = 500;
    
    % Verificar si el texto supera el límite
    if strlength(app.RespuestaTextArea.Value) > max_length
        app.RespuestaTextArea.Value = extractBefore(app.RespuestaTextArea.Value, max_length);
    end
    
    % Forzar el desplazamiento automático al final del texto
    if iscell(app.RespuestaTextArea.Value)
        app.RespuestaTextArea.Value = app.RespuestaTextArea.Value; % Refresca el TextArea
    end


        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Get the file path for locating images
            pathToMLAPP = fileparts(mfilename('fullpath'));

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [1 1 1];
            app.UIFigure.Position = [0 0 922 950];
            app.UIFigure.Name = 'MATLAB App';

            % Create TabGroup
            app.TabGroup = uitabgroup(app.UIFigure);
            app.TabGroup.Position = [2 -57 922 1008];

            % Create SimulinkTab
            app.SimulinkTab = uitab(app.TabGroup);
            app.SimulinkTab.Title = 'Simulink';
            app.SimulinkTab.BackgroundColor = 'none';

            % Create CARGARTXTButton
            app.CARGARTXTButton = uibutton(app.SimulinkTab, 'push');
            app.CARGARTXTButton.ButtonPushedFcn = createCallbackFcn(app, @CARGARTXTButtonPushed, true);
            app.CARGARTXTButton.FontWeight = 'bold';
            app.CARGARTXTButton.Position = [112 466 135 53];
            app.CARGARTXTButton.Text = 'CARGAR TXT';

            % Create SimulationControls
            app.SimulationControls = uisimcontrols(app.SimulinkTab);
            app.SimulationControls.Simulation = app.Simulation;
            app.SimulationControls.Position = [342 446 207 93];

            % Create MOVERButton_3
            app.MOVERButton_3 = uibutton(app.SimulinkTab, 'push');
            app.MOVERButton_3.ButtonPushedFcn = createCallbackFcn(app, @MOVERButton_3Pushed, true);
            app.MOVERButton_3.FontWeight = 'bold';
            app.MOVERButton_3.Position = [628 466 137 53];
            app.MOVERButton_3.Text = 'MOVER';

            % Create DETENERButton_3
            app.DETENERButton_3 = uibutton(app.SimulinkTab, 'push');
            app.DETENERButton_3.ButtonPushedFcn = createCallbackFcn(app, @DETENERButton_3Pushed, true);
            app.DETENERButton_3.BackgroundColor = [1 0.4118 0.1608];
            app.DETENERButton_3.FontWeight = 'bold';
            app.DETENERButton_3.Position = [548 349 125 53];
            app.DETENERButton_3.Text = 'DETENER';

            % Create ESPACIODESIMULACIONLabel
            app.ESPACIODESIMULACIONLabel = uilabel(app.SimulinkTab);
            app.ESPACIODESIMULACIONLabel.HorizontalAlignment = 'center';
            app.ESPACIODESIMULACIONLabel.FontName = 'C059';
            app.ESPACIODESIMULACIONLabel.FontSize = 34;
            app.ESPACIODESIMULACIONLabel.Position = [85 881 752 85];
            app.ESPACIODESIMULACIONLabel.Text = 'ESPACIO DE SIMULACION';

            % Create Button_4
            app.Button_4 = uibutton(app.SimulinkTab, 'push');
            app.Button_4.Icon = fullfile(pathToMLAPP, 'robotan5.png');
            app.Button_4.FontColor = [1 1 1];
            app.Button_4.Position = [321 610 263 264];
            app.Button_4.Text = '';

            % Create RESETButton
            app.RESETButton = uibutton(app.SimulinkTab, 'push');
            app.RESETButton.ButtonPushedFcn = createCallbackFcn(app, @RESETButtonPushed, true);
            app.RESETButton.BackgroundColor = [0.8 0.8 0.8];
            app.RESETButton.FontWeight = 'bold';
            app.RESETButton.Position = [218 349 125 53];
            app.RESETButton.Text = 'RESET';

            % Create RespuestaTextAreaLabel
            app.RespuestaTextAreaLabel = uilabel(app.SimulinkTab);
            app.RespuestaTextAreaLabel.HorizontalAlignment = 'right';
            app.RespuestaTextAreaLabel.FontSize = 18;
            app.RespuestaTextAreaLabel.FontWeight = 'bold';
            app.RespuestaTextAreaLabel.Position = [404 306 96 23];
            app.RespuestaTextAreaLabel.Text = 'Respuesta';

            % Create RespuestaTextArea
            app.RespuestaTextArea = uitextarea(app.SimulinkTab);
            app.RespuestaTextArea.ValueChangedFcn = createCallbackFcn(app, @RespuestaTextAreaValueChanged, true);
            app.RespuestaTextArea.Position = [112 42 682 265];

            % Create SIMULARLabel
            app.SIMULARLabel = uilabel(app.SimulinkTab);
            app.SIMULARLabel.FontName = 'C059';
            app.SIMULARLabel.FontSize = 18;
            app.SIMULARLabel.FontWeight = 'bold';
            app.SIMULARLabel.Position = [389 538 207 42];
            app.SIMULARLabel.Text = 'SIMULAR';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Interfaz_App_Designer_Simulacion

            % Associate the Simulink Model
            app.Simulation = simulation('simulador_trayectorias_AN5');

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)
            global myApp;
            myApp = app;


            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end