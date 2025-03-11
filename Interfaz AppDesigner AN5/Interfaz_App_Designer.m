classdef Interfaz_App_Designer < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                       matlab.ui.Figure
        TabGroup                       matlab.ui.container.TabGroup
        ArticularTab                   matlab.ui.container.Tab
        RESETButton_2                  matlab.ui.control.Button
        CARGARGEMELODIGITALLabel_2     matlab.ui.control.Label
        Button_CARGAR_GEMELO_DIGITAL   matlab.ui.control.Button
        EditFieldJoint5                matlab.ui.control.NumericEditField
        ValorLabel_6                   matlab.ui.control.Label
        EditFieldJoint1                matlab.ui.control.NumericEditField
        ValorLabel_5                   matlab.ui.control.Label
        EditFieldJoint6                matlab.ui.control.NumericEditField
        ValorLabel_4                   matlab.ui.control.Label
        EditFieldJoint4                matlab.ui.control.NumericEditField
        ValorLabel_3                   matlab.ui.control.Label
        EditFieldJoint3                matlab.ui.control.NumericEditField
        ValorLabel_2                   matlab.ui.control.Label
        EditFieldJoint2                matlab.ui.control.NumericEditField
        ValorLabel                     matlab.ui.control.Label
        SliderJoint1                   matlab.ui.control.Slider
        J1Label                        matlab.ui.control.Label
        EditFieldSpeed                 matlab.ui.control.NumericEditField
        SlidervalLabel_4               matlab.ui.control.Label
        J1Label_7                      matlab.ui.control.Label
        J1Label_6                      matlab.ui.control.Label
        J1Label_5                      matlab.ui.control.Label
        J1Label_4                      matlab.ui.control.Label
        J1Label_3                      matlab.ui.control.Label
        J1Label_2                      matlab.ui.control.Label
        ESPACIOARTICULARLabel          matlab.ui.control.Label
        Led_manu                       matlab.ui.control.Lamp
        AutoManualSwitch_3             matlab.ui.control.Switch
        DETENERButton_4                matlab.ui.control.Button
        MOVERButton                    matlab.ui.control.Button
        SliderJoint6                   matlab.ui.control.Slider
        SliderJoint5                   matlab.ui.control.Slider
        SliderJoint4                   matlab.ui.control.Slider
        SliderJoint3                   matlab.ui.control.Slider
        SliderJoint2                   matlab.ui.control.Slider
        SliderSpeed                    matlab.ui.control.Slider
        CartesianoTab                  matlab.ui.container.Tab
        RESETButton                    matlab.ui.control.Button
        CLEARButton                    matlab.ui.control.Button
        z_Lim0720Label                 matlab.ui.control.Label
        y_Lim500500Label               matlab.ui.control.Label
        x_Lim830320Label               matlab.ui.control.Label
        CARGARGEMELODIGITALLabel       matlab.ui.control.Label
        Button_CARGAR_GEMELO_DIGITAL2  matlab.ui.control.Button
        ESPACIOCARTESIANOLabel         matlab.ui.control.Label
        DETENERButton_2                matlab.ui.control.Button
        EditFieldSpeed_2               matlab.ui.control.NumericEditField
        SlidervalLabel_5               matlab.ui.control.Label
        VelocidadLabel                 matlab.ui.control.Label
        SliderSpeed_2                  matlab.ui.control.Slider
        Led_Manual                     matlab.ui.control.Lamp
        AutoManualSwitch_2             matlab.ui.control.Switch
        PosicionesCartesianasTextArea  matlab.ui.control.TextArea
        PosicionesCartesianasTextAreaLabel  matlab.ui.control.Label
        RzEditField                    matlab.ui.control.NumericEditField
        RzEditFieldLabel               matlab.ui.control.Label
        RyEditField                    matlab.ui.control.NumericEditField
        RyEditFieldLabel               matlab.ui.control.Label
        RxEditField                    matlab.ui.control.NumericEditField
        RxEditFieldLabel               matlab.ui.control.Label
        ZEditField                     matlab.ui.control.NumericEditField
        ZEditFieldLabel                matlab.ui.control.Label
        YEditField                     matlab.ui.control.NumericEditField
        YEditFieldLabel                matlab.ui.control.Label
        XEditField                     matlab.ui.control.NumericEditField
        XEditFieldLabel                matlab.ui.control.Label
        MOVERButton_2                  matlab.ui.control.Button
        AADIRButton                    matlab.ui.control.Button
        SimulinkTab                    matlab.ui.container.Tab
        SIMULARLabel                   matlab.ui.control.Label
        RESETButton_3                  matlab.ui.control.Button
        Button_4                       matlab.ui.control.Button
        ESPACIODESIMULACIONLabel       matlab.ui.control.Label
        DETENERButton_3                matlab.ui.control.Button
        MOVERButton_3                  matlab.ui.control.Button
        SimulationControls             simulink.ui.control.SimulationControls
        CARGARTXTButton                matlab.ui.control.Button
    end


    % Public properties that correspond to the Simulink model
    properties (Access = public, Transient)
        Simulation simulink.Simulation
    end

properties (Access = private)
        robot                % Modelo del robot utilizando rigidBodyTree
        node                 % Nodo ROS2 para la comunicación
        sub_current_joint_position   % Suscriptor para el tópico 'current_joint_position'
        sub_output_cartesian_position % Suscriptor para el tópico 'output_cartesian_position'
        sub_output_joint_position     % Suscriptor para el tópico 'output_joint_position'
        sub_inversa_output   % Suscriptor para el tópico 'output_joint_position'
        pub_api_command      % Publicador para el tópico 'api_command'
        pub_input_joint_position      % Publicador para el tópico 'input_joint_position'
        pub_input_cartesian_position  % Publicador para el tópico 'input_cartesian_position'
        pub_input_cartesian_path      % Publicador para el tópico 'input_cartesian_path'
        initialguess         % Configuración inicial del robot
        ax                   % Ejes para la visualización 3D del robot
        isArticularMode = false % Indicador de modo articular activo
        fig                  % Figura para la visualización 3D
        lastJointPositions   % Almacena las últimas posiciones de las articulaciones
        updateThreshold = 0.2; % Umbral de actualización en grados para cambios de articulación
        % Propiedades para almacenar posiciones y rotaciones
        positions = [];      % Matriz para almacenar las posiciones cartesianas
        rotations = [];      % Matriz para almacenar las rotaciones (roll, pitch, yaw)
        num_points = 0;      % Contador de puntos añadidos para movimientos
        configs_matrix = []; % Matriz para almacenar configuraciones articulares calculadas
        posicionesTxtFile    % Ruta del archivo .txt cargado con posiciones
        publisher            % Publicador adicional para comandos específicos
        generatedFigures = []; % Almacena las figuras generadas por la interfaz
       lastIKAngles = [];  % Para almacenar la última solución IK y evitar duplicados
    end

    methods (Access = private)
        % Código que se ejecuta después de la creación del componente

        %% Función para iniciar la simulación y vincular el progreso
        function startSimulation(app)
            % Inicializa el objeto de simulación y lo vincula al control de progreso
            app.Simulation = simulacion('simulador_trayectorias_AN5'); % Carga la simulación específica
            app.SimulationProgress.Simulation = app.Simulation; % Asocia el progreso de la simulación

            % Escucha cambios en el progreso de la simulación para actualizar el tiempo mostrado
            addlistener(app.SimulationProgress, 'ProgressChanged', @(~,~) app.updateSimulationTime());

            % Inicia la ejecución de la simulación
            start(app.Simulation);
        end

        %% Función para actualizar el tiempo de simulación en el Label
        function updateSimulationTime(app)
            % Recupera el tiempo actual de la simulación
            simTime = app.SimulationProgress.Simulation.getSimulationTime();
            % Actualiza el texto del Label con el tiempo de simulación
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
                % Definición de límites adicionales si es necesario
                hold(app.ax, 'off');
                rotate3d(app.ax, 'on'); % Habilita la rotación interactiva
                app.initialguess = app.robot.homeConfiguration; % Configuración inicial
                disp('Modelo URDF cargado y visualizado con ajustes de vista.');
            end
        end

        %% Función para manejar el cierre de la figura de visualización
        function onCloseVisualizationFigure(app)
            % Cierra la figura de visualización y limpia referencias
            if ~isempty(app.fig) && isvalid(app.fig)
                try
                    % Detiene cualquier suscripción activa antes de cerrar
                    app.stopJointPositionSubscriber();

                    % Elimina la figura y resetea las referencias
                    delete(app.fig);
                    app.fig = [];  % Limpia el objeto de la figura
                    app.ax = [];   % Limpia el objeto de los ejes
                    disp('Figura de visualización cerrada.');
                catch ME
                    % Muestra un mensaje de error si ocurre algún problema
                    disp(['Error al cerrar la figura: ', ME.message]);
                end
            else
                disp('La figura ya está cerrada o no es válida.');
            end
        end

        %% Función para cerrar solo las figuras generadas por la interfaz
        function closeGeneratedFigures(app)
            % Cierra todas las figuras que fueron generadas por la interfaz
            if ~isempty(app.generatedFigures)
                for i = 1:length(app.generatedFigures)
                    if isvalid(app.generatedFigures(i))
                        delete(app.generatedFigures(i));
                    end
                end
                app.generatedFigures = []; % Limpia la lista de figuras
                disp('Todas las figuras generadas por la interfaz han sido cerradas.');
            end
        end

        %% Función para actualizar el robot con una sola articulación
        function updateRobotSingleJoint(app, jointIndex, jointAngle)
            % Actualiza la posición de una articulación específica y refresca la visualización
            configSol = app.initialguess;

            % Asigna el nuevo ángulo a la articulación indicada
            configSol(jointIndex).JointPosition = deg2rad(jointAngle);

            % Recupera los ángulos actuales de las articulaciones desde los campos de edición
            jointAngles = [
                app.EditFieldJoint1.Value, ...
                app.EditFieldJoint2.Value, ...
                app.EditFieldJoint3.Value, ...
                app.EditFieldJoint4.Value, ...
                app.EditFieldJoint5.Value, ...
                app.EditFieldJoint6.Value];

            % Actualiza las demás articulaciones con sus ángulos actuales
            for i = 1:length(jointAngles)
                if i ~= jointIndex
                    configSol(i).JointPosition = deg2rad(jointAngles(i));
                end
            end

            % Limpia los ejes antes de dibujar la nueva configuración
            cla(app.ax);

            % Muestra el robot en la nueva configuración
            show(app.robot, configSol, 'Parent', app.ax);
            axis(app.ax, 'vis3d');

            % Configura la vista y los límites de los ejes
            view(app.ax, [18 20]);
            app.ax.XLim = [-1 1];
            app.ax.YLim = [-1 1];
            app.ax.ZLim = [0 1.2];
            hold(app.ax, 'off');

            rotate3d(app.ax, 'on'); % Habilita la rotación interactiva
        end

        %% Función para actualizar el robot desde los sliders
        function updateRobotFromSliders(app)
            % Actualiza la visualización del robot basado en los valores actuales de los sliders
            if ~isempty(app.ax) && isvalid(app.ax)
                cla(app.ax); % Limpia el contenido de los ejes

                % Obtiene la configuración del robot basada en los sliders
                configSol = app.initialguess;
                jointAngles = [
                    app.SliderJoint1.Value, ...
                    app.SliderJoint2.Value, ...
                    app.SliderJoint3.Value, ...
                    app.SliderJoint4.Value, ...
                    app.SliderJoint5.Value, ...
                    app.SliderJoint6.Value];

                % Asigna los ángulos de las articulaciones a la configuración
                for i = 1:length(jointAngles)
                    configSol(i).JointPosition = deg2rad(jointAngles(i));
                end

                % Muestra el robot en la nueva configuración
                show(app.robot, configSol, 'Parent', app.ax);
                axis(app.ax, 'vis3d');

                % Configura la vista y los límites de los ejes
                view(app.ax, [18 20]);
                app.ax.XLim = [-1 1];
                app.ax.YLim = [-1 1];
                app.ax.ZLim = [0 1.2];
                hold(app.ax, 'off');

                rotate3d(app.ax, 'on'); % Habilita la rotación interactiva
            else
                disp('Advertencia: No hay un eje válido para la visualización.');
            end
        end

        %% Función para actualizar el robot desde los ángulos articulares
        function updateRobotFromJointAngles(app, jointAngles)
            % Actualiza la visualización del robot con los ángulos articulares proporcionados
            if ~isempty(app.ax) && isvalid(app.ax)
                cla(app.ax); % Limpia el contenido de los ejes

                % Asigna los ángulos articulares a la configuración del robot
                configSol = app.initialguess;
                for i = 1:length(jointAngles)
                    configSol(i).JointPosition = deg2rad(jointAngles(i));
                end

                % Muestra el robot en la nueva configuración
                show(app.robot, configSol, 'Parent', app.ax);
                axis(app.ax, 'vis3d');
                view(app.ax, [18 20]);
                app.ax.XLim = [-1 1];
                app.ax.YLim = [-1 1];
                app.ax.ZLim = [0 1.2];
                hold(app.ax, 'off');
                rotate3d(app.ax, 'on'); % Habilita la rotación interactiva
            else
                disp('Advertencia: No hay un eje válido para la visualización.');
            end
        end

        %% Función para iniciar el suscriptor de posiciones de articulaciones
        function startJointPositionSubscriber(app)
            % Inicia la suscripción al tópico 'current_joint_position' para recibir posiciones de articulaciones
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
            % Detiene y elimina la suscripción a posiciones de articulaciones si está activa
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
            % Procesa los mensajes recibidos en el tópico 'current_joint_position'
            jointPositions = str2double(strsplit(msg.data, ','));

            % Verifica que se recibieron exactamente 6 valores y que no está en modo articular
            if length(jointPositions) == 6 && ~app.isArticularMode
                % Actualiza los sliders y campos de edición con las posiciones recibidas
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

                % Actualiza la visualización del robot basado en los sliders
                app.updateRobotFromSliders();
            end
        end

function inversaOutputCallback(app, msg)
    try
        % Convertir el string recibido en 6 números
        jointAngles = str2double(strsplit(msg.data, ','));

        if length(jointAngles) == 6 && all(~isnan(jointAngles))

            % --- Evitar repeticiones ---
            % Si la App ya había recibido la misma solución hace poco,
            % la ignoramos. Usamos tolerancia de 0.01 para flotantes.
            if ~isempty(app.lastIKAngles)
                if norm(jointAngles - app.lastIKAngles) < 0.01
                    % Es la misma solución (o casi), ignoramos
                    disp('Se recibió la misma solución IK repetida. Se ignora.');
                    return;
                end
            end
            % Guardamos esta nueva solución en lastIKAngles
            app.lastIKAngles = jointAngles;

            % --- Actualiza los campos de edición (J1..J6) y sliders ---
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

            % --- Visualizar el robot con estos ángulos en la figura 3D ---
            app.updateRobotFromJointAngles(jointAngles);

            % --- Guardar esta solución en configs_matrix (para "MOVER") ---
            app.configs_matrix = [app.configs_matrix; jointAngles];

            % --- Añadir al TextArea, redondeado a 2 decimales ---
            jointAngles2dec = round(jointAngles, 2);
            % Formato: "Articulares:[ xx.xx, yy.yy, ... ]"
            newIKLine = sprintf('Articulares:[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]', ...
                                jointAngles2dec(1), jointAngles2dec(2), jointAngles2dec(3), ...
                                jointAngles2dec(4), jointAngles2dec(5), jointAngles2dec(6));

            current_text = app.PosicionesCartesianasTextArea.Value;
            app.PosicionesCartesianasTextArea.Value = [current_text; {newIKLine}];

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
            % Configura el estado inicial de los switches y LEDs en modo automático
            app.AutoManualSwitch.Value = 'Automatico';
            app.AutoManualSwitch_2.Value = 'Automatico';

            % Envía el comando para establecer el modo automático
            app.sendDragTeachSwitch(0);

            % Actualiza el estado de los LEDs para reflejar el modo automático
            app.Led_manu.Color = [0 1 0];  % LED manual verde
            app.Led_Auto.Color = [1 1 1];  % LED automático apagado
            app.Led_Manual.Color = [0 1 0];  % LED manual verde
            app.Led_Auto.Color = [1 1 1];  % LED automático apagado
        end

        %% Función para enviar el comando DragTeachSwitch
        function sendDragTeachSwitch(app, mode)
            % Envía comandos al tópico 'api_command' para cambiar entre modos manual y automático
            if isempty(app.node)
                app.node = ros2node('matlab_pub_node');
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            % Crea el mensaje ROS
            msg = ros2message(app.pub_api_command);

            % Comandos para el modo manual (0)
            if mode == 0
                % Enviar el comando DragTeachSwitch con modo manual
                msg.data = sprintf('DragTeachSwitch(%d)', mode);
                send(app.pub_api_command, msg);

                % Enviar el comando SplineEnd
                msg.data = 'SplineEnd()';
                send(app.pub_api_command, msg);

                % Enviar el comando ResetAllError
                msg.data = 'ResetAllError()';
                send(app.pub_api_command, msg);

                % Enviar los comandos StartJOG para ambas articulaciones
                msg.data = 'StartJOG(0,6,0,100)';
                send(app.pub_api_command, msg);

                msg.data = 'StartJOG(1,6,0,100)';
                send(app.pub_api_command, msg);

                disp(['Comando enviado: ', msg.data]);
            end

            % Comandos para el modo automático (1)
            if mode == 1
                % Enviar el comando DragTeachSwitch con modo automático
                msg.data = sprintf('DragTeachSwitch(%d)', mode);
                send(app.pub_api_command, msg);

                disp(['Comando enviado: ', msg.data]);
            end
        end

        %% Función para enviar la posición cartesiana
        function sendCartesianPosition(app)
            % Envía la posición cartesiana actual al tópico de cinemática inversa
            x = app.XEditField.Value;
            y = app.YEditField.Value;
            z = app.ZEditField.Value;
            rx = app.RxEditField.Value;
            ry = app.RyEditField.Value;
            rz = app.RzEditField.Value;

            % Formatea la cadena del mensaje con los valores actuales
            input_str = sprintf('%g,%g,%g,%g,%g,%g', x, y, z, rx, ry, rz);

            % Publica el mensaje al tópico de entrada de cinemática inversa
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
            % Configura los nodos, publicadores y suscriptores necesarios para ROS2
            if isempty(app.node)
                app.node = ros2node('matlab_node2');
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            if isempty(app.sub_output_joint_position)
                app.sub_output_joint_position = ros2subscriber(app.node, 'output_joint_position', 'std_msgs/String', @app.inversaOutputCallback);
            end

            if isempty(app.pub_input_cartesian_path)
                app.pub_input_cartesian_path = ros2publisher(app.node, 'input_cartesian_path', 'std_msgs/String');
            end
        end
end

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: MOVERButton
        function MOVERButtonPushed(app, event)
% Leer los valores de los sliders
            jointAngles = [
                app.SliderJoint1.Value, ...
                app.SliderJoint2.Value, ...
                app.SliderJoint3.Value, ...
                app.SliderJoint4.Value, ...
                app.SliderJoint5.Value, ...
                app.SliderJoint6.Value];

            % Leer el valor del slider de velocidad
            speed = round(app.SliderSpeed.Value);

            % Inicializar nodo y publicador ROS si no existen
            if isempty(app.node)
                app.node = ros2node('matlab_pub_node'); % Nombre del nodo ROS
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            % Crear mensaje ROS
            msg = ros2message(app.pub_api_command);

            % Enviar el comando SplineStart
            msg.data = 'SplineStart()';
            send(app.pub_api_command, msg);
            disp(['Publicado: ', msg.data]);

            % Enviar el comando JNTPoint con los ángulos de las articulaciones
            commandJNTPoint = sprintf('JNTPoint(1,%g,%g,%g,%g,%g,%g)', jointAngles);
            msg.data = commandJNTPoint;
            send(app.pub_api_command, msg);
            disp(['Publicado: ', commandJNTPoint]);

            % Enviar el comando SplinePTP con la velocidad seleccionada
            commandSplinePTP = sprintf('SplinePTP(JNT1,%g)', speed);
            msg.data = commandSplinePTP;
            send(app.pub_api_command, msg);
            disp(['Publicado: ', commandSplinePTP]);

            % Enviar el comando SplineEnd
            msg.data = 'SplineEnd()';
            send(app.pub_api_command, msg);
            disp(['Publicado: ', msg.data]);

            % Reactivar la suscripción para actualizar el URDF en tiempo real
            app.isArticularMode = false;
            app.startJointPositionSubscriber();
        end

        % Value changing function: SliderJoint1
        function SliderJoint1ValueChanging(app, event)
          app.EditFieldJoint1.Value = event.Value;
            app.updateRobotSingleJoint(1, event.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changing function: SliderJoint2
        function SliderJoint2ValueChanging(app, event)
           app.EditFieldJoint2.Value = event.Value;
            app.updateRobotSingleJoint(2, event.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changing function: SliderJoint3
        function SliderJoint3ValueChanging(app, event)
           app.EditFieldJoint3.Value = event.Value;
            app.updateRobotSingleJoint(3, event.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changing function: SliderJoint4
        function SliderJoint4ValueChanging(app, event)
            app.EditFieldJoint4.Value = event.Value;
            app.updateRobotSingleJoint(4, event.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changing function: SliderJoint5
        function SliderJoint5ValueChanging(app, event)
            app.EditFieldJoint5.Value = event.Value;
            app.updateRobotSingleJoint(5, event.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changing function: SliderJoint6
        function SliderJoint6ValueChanging(app, event)
            app.EditFieldJoint6.Value = event.Value;
            app.updateRobotSingleJoint(6, event.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: EditFieldJoint1
        function EditFieldJoint1ValueChanged(app, event)
             app.SliderJoint1.Value = app.EditFieldJoint1.Value;
            app.updateRobotSingleJoint(1, app.EditFieldJoint1.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: EditFieldJoint2
        function EditFieldJoint2ValueChanged(app, event)
             app.SliderJoint2.Value = app.EditFieldJoint2.Value;
            app.updateRobotSingleJoint(2, app.EditFieldJoint2.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: EditFieldJoint3
        function EditFieldJoint3ValueChanged(app, event)
            app.SliderJoint3.Value = app.EditFieldJoint3.Value;
            app.updateRobotSingleJoint(3, app.EditFieldJoint3.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: EditFieldJoint4
        function EditFieldJoint4ValueChanged(app, event)
            app.SliderJoint4.Value = app.EditFieldJoint4.Value;
            app.updateRobotSingleJoint(4, app.EditFieldJoint4.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: EditFieldJoint5
        function EditFieldJoint5ValueChanged(app, event)
             app.SliderJoint5.Value = app.EditFieldJoint5.Value;
            app.updateRobotSingleJoint(5, app.EditFieldJoint5.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: EditFieldJoint6
        function EditFieldJoint6ValueChanged(app, event)
           app.SliderJoint6.Value = app.EditFieldJoint6.Value;
            app.updateRobotSingleJoint(6, app.EditFieldJoint6.Value);
            % Activar modo articular
            app.isArticularMode = true;
            app.stopJointPositionSubscriber();
        end

        % Value changed function: SliderSpeed
        function SliderSpeedValueChanged(app, event)
 % Cuando el slider cambia, actualizar el EditField solo si no es igual al valor actual
    newValue = event.Value;
    if app.EditFieldSpeed.Value ~= newValue
        app.EditFieldSpeed.Value = newValue;
    end
        end

        % Value changed function: EditFieldSpeed
        function EditFieldSpeedValueChanged(app, event)
            % Cuando el EditField cambia, actualizar el Slider solo si no es igual al valor actual
    newValue = event.Value;
    if app.SliderSpeed.Value ~= newValue
        app.SliderSpeed.Value = newValue;
    end

        end

        % Button pushed function: DETENERButton_4
        function DETENERButtonPushed(app, event)
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

        % Button pushed function: AADIRButton
        function AADIRButtonPushed(app, event)
% Obtener los valores de los campos de edición
    x  = app.XEditField.Value;
    y  = app.YEditField.Value;
    z  = app.ZEditField.Value;
    rx = app.RxEditField.Value;
    ry = app.RyEditField.Value;
    rz = app.RzEditField.Value;

    % Definir límites
    x_lim = [-830, -320]; 
    y_lim = [-500, 500];  
    z_lim = [0, 720];     
    rx_lim_1 = [-180, -20]; 
    rx_lim_2 = [20, 180];

    % Verificar si las posiciones y orientaciones están dentro del rango
    if  (x < x_lim(1) || x > x_lim(2)) || ...
        (y < y_lim(1) || y > y_lim(2)) || ...
        (z < z_lim(1) || z > z_lim(2)) || ...
        ( ~((rx >= rx_lim_1(1) && rx <= rx_lim_1(2)) || (rx >= rx_lim_2(1) && rx <= rx_lim_2(2))) )
       
        % Posición fuera de rango, mostrar mensaje y salir
        current_text = app.PosicionesCartesianasTextArea.Value;
        app.PosicionesCartesianasTextArea.Value = [current_text; {'Posiciones fuera del rango'}];
        return;  
    end

    % Crear la cadena con los 6 valores (X,Y,Z,Rx,Ry,Rz)
    input_str = sprintf('%g,%g,%g,%g,%g,%g', x, y, z, rx, ry, rz);

    % Publicar el mensaje en /input_cartesian_position para que el nodo calcule la IK
    if isempty(app.pub_input_cartesian_position)
        app.pub_input_cartesian_position = ros2publisher(app.node, ...
            'input_cartesian_position', 'std_msgs/String');
    end

    msg = ros2message(app.pub_input_cartesian_position);
    msg.data = input_str;
    send(app.pub_input_cartesian_position, msg);

    disp(['Posición y rotación añadidas y enviadas para cálculo de la inversa: ', input_str]);

    % Incrementar el contador de puntos (usado al pulsar "MOVER")
    app.num_points = app.num_points + 1;

    % Guardar la posición/rotación para tu uso (opcional)
    app.positions = [app.positions; x, y, z];
    app.rotations = [app.rotations; rx, ry, rz];

    % Mostrar la nueva posición en el TextArea
    new_position_str = sprintf('X:%g, Y:%g, Z:%g, Rx:%g, Ry:%g, Rz:%g', ...
                               x, y, z, rx, ry, rz);
    current_text = app.PosicionesCartesianasTextArea.Value;
    app.PosicionesCartesianasTextArea.Value = [current_text; {new_position_str}];

       
        end

        % Button pushed function: MOVERButton_2
        function MOVERButton_2Pushed(app, event)
 % Verificar que se hayan calculado configuraciones articulares
            if isempty(app.configs_matrix) || app.num_points == 0
                disp('No hay configuraciones articulares almacenadas.');
                return;
            end

            % Inicializar el nodo y el publicador ROS2 si no están ya inicializados
            if isempty(app.node)
                app.node = ros2node('/matlab_command_node'); % Nombre del nodo ROS
            end

            if isempty(app.pub_api_command)
                app.pub_api_command = ros2publisher(app.node, 'api_command', 'std_msgs/String');
            end

            % Obtener el valor de la velocidad del slider
            speed2 = round(app.SliderSpeed_2.Value);

            % Enviar todos los puntos almacenados con JNTPoint
            for i = 1:app.num_points
                jointAngles = app.configs_matrix(i, :);

                % Crear el comando JNTPoint para cada configuración
                cmd_str = sprintf('JNTPoint(%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)', ...
                                  i, jointAngles(1), jointAngles(2), jointAngles(3), ...
                                  jointAngles(4), jointAngles(5), jointAngles(6));
                msg = ros2message(app.pub_api_command);
                msg.data = cmd_str;
                send(app.pub_api_command, msg);
                disp(['Publicado: ', cmd_str]);
            end

            % Iniciar movimiento spline en ROS
            msg = ros2message(app.pub_api_command);
            msg.data = 'SplineStart()';
            send(app.pub_api_command, msg);
            disp('Publicado: SplineStart()');

            % Enviar los comandos SplinePTP para cada JNTPoint
            for i = 1:app.num_points
                cmd_str = sprintf('SplinePTP(JNT%d,%d)', i, speed2);
                msg = ros2message(app.pub_api_command);
                msg.data = cmd_str;
                send(app.pub_api_command, msg);
                disp(['Publicado: ', cmd_str]);
            end

            % Resetear contador de puntos y limpiar matrices
            app.num_points = 0;
            app.positions = [];
            app.rotations = [];
            app.configs_matrix = [];

            % Limpiar el área de texto de posiciones cartesianas
            app.PosicionesCartesianasTextArea.Value = '';

        end

        % Value changed function: AutoManualSwitch_3
        function AutoManualSwitchValueChanged(app, event)

        value = app.AutoManualSwitch_3.Value;  % Cambiado a AutoManualSwitch_3
        disp(['Valor del switch: ', value]);  % Mostrar el valor para depuración

        try
            if strcmp(value, 'Automatico')
                % Enviar comando para modo manual
                app.sendDragTeachSwitch(0);  % Modo manual

                % Actualizar LEDs
                app.Led_manu.Color = [0 1 0];  % LED manual verde
                app.Led_Auto.Color = [1 1 1];  % Apagar LED automático (blanco)
            else
                % Enviar comando para modo automático
                app.sendDragTeachSwitch(1);  % Modo automático

                % Actualizar LEDs
                app.Led_manu.Color = [1 1 1];  % Apagar LED manual (blanco)
                app.Led_Auto.Color = [0 0 1];  % LED automático azul
            end
        catch ME
            disp(['Error al cambiar el modo: ', ME.message]);  % Mostrar cualquier error
        end
   
        end

        % Size changed function: CartesianoTab
        function CartesianoTabSizeChanged(app, event)
            position = app.CartesianoTab.Position;
            
        end

        % Value changed function: PosicionesCartesianasTextArea
        function PosicionesCartesianasTextAreaValueChanged(app, event)
    % Límite de caracteres
    max_length = 500;
    
    % Toma el valor actual
    currentVal = app.PosicionesCartesianasTextArea.Value;
    
    if iscell(currentVal)
        textoUnido = strjoin(currentVal, newline);
    else
        textoUnido = string(currentVal); 
    end
    
    if strlength(textoUnido) > max_length
        textoUnido = extractBefore(textoUnido, max_length);
    end
    
    app.PosicionesCartesianasTextArea.Value = textoUnido;
        end

        % Value changed function: SliderSpeed_2
        function SliderSpeed_2ValueChanged(app, event)
            newValue = event.Value;
    if app.EditFieldSpeed_2.Value ~= newValue
        app.EditFieldSpeed_2.Value = newValue;
    end
            
            
        end

        % Value changed function: EditFieldSpeed_2
        function EditFieldSpeed_2ValueChanged(app, event)
                    % Cuando el EditField cambia, actualizar el Slider solo si no es igual al valor actual
    newValue = event.Value;
    if app.SliderSpeed_2.Value ~= newValue
        app.SliderSpeed_2.Value = newValue;
    end
        end

        % Value changed function: AutoManualSwitch_2
        function AutoManualSwitch_2ValueChanged(app, event)
           
 value = app.AutoManualSwitch_2.Value;  % Cambiado a AutoManualSwitch_2
        disp(['Valor del switch: ', value]);  % Mostrar el valor para depuración

        try
            if strcmp(value, 'Automatico')
                % Enviar comando para modo manual
                app.sendDragTeachSwitch(0);  % Modo manual

                % Actualizar LEDs
                app.Led_Manual.Color = [0 1 0];  % LED manual verde
                app.Led_Auto.Color = [1 1 1];  % Apagar LED automático (blanco)
            else
                % Enviar comando para modo automático
                app.sendDragTeachSwitch(1);  % Modo automático

                % Actualizar LEDs
                app.Led_Manual.Color = [1 1 1];  % Apagar LED manual (blanco)
                app.Led_Auto.Color = [0 0 1];  % LED automático azul
            end
        catch ME
            disp(['Error al cambiar el modo: ', ME.message]);  % Mostrar cualquier error
        end
        end

        % Button pushed function: DETENERButton_2
        function DETENERButton_2Pushed(app, event)
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

        % Button pushed function: CARGARTXTButton
        function CARGARTXTButtonPushed(app, event)
  % Seleccionar el archivo .txt
            [file, path] = uigetfile('*.txt', 'Seleccione el archivo de posiciones');

            if isequal(file, 0)
                disp('No se seleccionó ningún archivo.');
                return;
            end

            % Cerrar todas las figuras generadas previamente
            app.closeGeneratedFigures();

            % Limpiar cualquier referencia a figuras en la lista de figuras generadas
            app.generatedFigures = [];
            close all;

            % Ruta completa del archivo seleccionado
            archivo = fullfile(path, file);
            disp(['Archivo seleccionado: ', archivo]);

            % Sobrescribir el archivo como desired_positions.txt
            archivo_destino = 'desired_positions.txt';
            copyfile(archivo, archivo_destino);
            disp(['Archivo cargado y guardado como: ', archivo_destino]);

            % Leer el archivo y calcular Tfinal
            data = dlmread(archivo_destino, ',');
            num_puntos = size(data, 1);
            Tfinal = num_puntos * 0.1;
            assignin('base', 'Tfinal', Tfinal);
            disp(['Número de puntos encontrados: ', num2str(num_puntos)]);
            disp(['Tfinal calculado: ', num2str(Tfinal), ' segundos']);
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

        % Button pushed function: Button_CARGAR_GEMELO_DIGITAL
        function Button_CARGAR_GEMELO_DIGITALPushed(app, event)
% Cierra todas las figuras generadas previamente
            app.closeGeneratedFigures();

            % Detener cualquier suscripción activa antes de cargar de nuevo
            app.stopJointPositionSubscriber();

            % Cargar el modelo URDF y visualización
            app.loadAndVisualizeURDF();

            % Verificar si la figura de visualización es válida para iniciar la suscripción
            if isvalid(app.fig)
                app.startJointPositionSubscriber();
            end

            % Almacenar la figura generada en generatedFigures para poder cerrarla más adelante
            app.generatedFigures = [app.generatedFigures, app.fig];

            % Establecer valores iniciales de velocidad
            app.SliderSpeed.Value = 10;
            app.EditFieldSpeed.Value = 10;
            app.SliderSpeed_2.Value = 10;
            app.EditFieldSpeed_2.Value = 10;

            % Inicializar el nodo y los publicadores/suscriptores ROS si no existen
            if isempty(app.node)
                app.node = ros2node('matlab_node2'); % Nombre del nodo ROS
            end

            if isempty(app.pub_input_cartesian_position)
                app.pub_input_cartesian_position = ros2publisher(app.node, 'input_cartesian_position', 'std_msgs/String');
            end

            if isempty(app.sub_output_joint_position)
                app.sub_output_joint_position = ros2subscriber(app.node, 'output_joint_position', 'std_msgs/String', @app.inversaOutputCallback);
            end

            disp('Modelo 3D cargado y suscripción ROS iniciada.');
       
        end

        % Button pushed function: Button_CARGAR_GEMELO_DIGITAL2
        function Button_CARGAR_GEMELO_DIGITAL2Pushed(app, event)
  % Cierra todas las figuras generadas previamente
            app.closeGeneratedFigures();

            % Detener cualquier suscripción activa antes de cargar de nuevo
            app.stopJointPositionSubscriber();

            % Cargar el modelo URDF y visualización
            app.loadAndVisualizeURDF();

            % Verificar si la figura de visualización es válida para iniciar la suscripción
            if isvalid(app.fig)
                app.startJointPositionSubscriber();
            end

            % Almacenar la figura generada en generatedFigures para poder cerrarla más adelante
            app.generatedFigures = [app.generatedFigures, app.fig];

            % Establecer valores iniciales de velocidad
            app.SliderSpeed.Value = 10;
            app.EditFieldSpeed.Value = 10;
            app.SliderSpeed_2.Value = 10;
            app.EditFieldSpeed_2.Value = 10;

            % Inicializar el nodo y los publicadores/suscriptores ROS si no existen
            if isempty(app.node)
                app.node = ros2node('matlab_node2'); % Nombre del nodo ROS
            end

            if isempty(app.pub_input_cartesian_position)
                app.pub_input_cartesian_position = ros2publisher(app.node, 'input_cartesian_position', 'std_msgs/String');
            end

            if isempty(app.sub_output_joint_position)
                app.sub_output_joint_position = ros2subscriber(app.node, 'output_joint_position', 'std_msgs/String', @app.inversaOutputCallback);
            end

            disp('Modelo 3D cargado y suscripción ROS iniciada.');
        end

        % Value changed function: XEditField
        function XEditFieldValueChanged(app, event)
            app.sendCartesianPosition();
        end

        % Value changed function: YEditField
        function YEditFieldValueChanged(app, event)
            app.sendCartesianPosition();
            
        end

        % Value changed function: ZEditField
        function ZEditFieldValueChanged(app, event)
            app.sendCartesianPosition();
            
        end

        % Value changed function: RxEditField
        function RxEditFieldValueChanged(app, event)
            app.sendCartesianPosition();
            
        end

        % Value changed function: RyEditField
        function RyEditFieldValueChanged(app, event)
            app.sendCartesianPosition();
            
        end

        % Value changed function: RzEditField
        function RzEditFieldValueChanged(app, event)
            app.sendCartesianPosition();
            
        end

        % Button pushed function: CLEARButton
        function CLEARButtonPushed(app, event)

    % Restablecer listas/matrices internas
    app.num_points = 0;
    app.positions = [];
    app.rotations = [];
    app.configs_matrix = [];

    % Dejar la ventana de texto vacía
    app.PosicionesCartesianasTextArea.Value = {''};

    disp('Se han borrado las posiciones almacenadas.');


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

        % Button pushed function: RESETButton_2
        function RESETButton_2Pushed(app, event)
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

        % Button pushed function: RESETButton_3
        function RESETButton_3Pushed(app, event)
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
            app.TabGroup.Position = [2 1 922 950];

            % Create ArticularTab
            app.ArticularTab = uitab(app.TabGroup);
            app.ArticularTab.Title = 'Articular';
            app.ArticularTab.BackgroundColor = 'none';

            % Create SliderSpeed
            app.SliderSpeed = uislider(app.ArticularTab);
            app.SliderSpeed.ValueChangedFcn = createCallbackFcn(app, @SliderSpeedValueChanged, true);
            app.SliderSpeed.FontColor = [1 0.4118 0.1608];
            app.SliderSpeed.Position = [121 351 159 3];

            % Create SliderJoint2
            app.SliderJoint2 = uislider(app.ArticularTab);
            app.SliderJoint2.Limits = [-180 0];
            app.SliderJoint2.ValueChangingFcn = createCallbackFcn(app, @SliderJoint2ValueChanging, true);
            app.SliderJoint2.FontColor = [1 0.4118 0.1608];
            app.SliderJoint2.Position = [119 657 150 3];

            % Create SliderJoint3
            app.SliderJoint3 = uislider(app.ArticularTab);
            app.SliderJoint3.Limits = [-150 150];
            app.SliderJoint3.ValueChangingFcn = createCallbackFcn(app, @SliderJoint3ValueChanging, true);
            app.SliderJoint3.FontColor = [1 0.4118 0.1608];
            app.SliderJoint3.Position = [119 511 150 3];

            % Create SliderJoint4
            app.SliderJoint4 = uislider(app.ArticularTab);
            app.SliderJoint4.Limits = [-265 85];
            app.SliderJoint4.ValueChangingFcn = createCallbackFcn(app, @SliderJoint4ValueChanging, true);
            app.SliderJoint4.FontColor = [1 0.4118 0.1608];
            app.SliderJoint4.Position = [426 783 150 3];

            % Create SliderJoint5
            app.SliderJoint5 = uislider(app.ArticularTab);
            app.SliderJoint5.Limits = [-175 175];
            app.SliderJoint5.ValueChangingFcn = createCallbackFcn(app, @SliderJoint5ValueChanging, true);
            app.SliderJoint5.FontColor = [1 0.4118 0.1608];
            app.SliderJoint5.Position = [425 657 150 3];

            % Create SliderJoint6
            app.SliderJoint6 = uislider(app.ArticularTab);
            app.SliderJoint6.Limits = [-179 179];
            app.SliderJoint6.MajorTicks = [-175 -105 -35 35 105 175];
            app.SliderJoint6.ValueChangingFcn = createCallbackFcn(app, @SliderJoint6ValueChanging, true);
            app.SliderJoint6.MinorTicks = [-175 -165 -155 -145 -135 -125 -115 -105 -95 -85 -75 -65 -55 -45 -35 -25 -15 -5 5 15 25 35 45 55 65 75 85 95 105 115 125 135 145 155 165 175];
            app.SliderJoint6.FontColor = [1 0.4118 0.1608];
            app.SliderJoint6.Position = [426 511 150 3];

            % Create MOVERButton
            app.MOVERButton = uibutton(app.ArticularTab, 'push');
            app.MOVERButton.ButtonPushedFcn = createCallbackFcn(app, @MOVERButtonPushed, true);
            app.MOVERButton.BackgroundColor = [1 1 1];
            app.MOVERButton.FontWeight = 'bold';
            app.MOVERButton.Position = [450 279 105 32];
            app.MOVERButton.Text = 'MOVER';

            % Create DETENERButton_4
            app.DETENERButton_4 = uibutton(app.ArticularTab, 'push');
            app.DETENERButton_4.ButtonPushedFcn = createCallbackFcn(app, @DETENERButtonPushed, true);
            app.DETENERButton_4.BackgroundColor = [1 0.4118 0.1608];
            app.DETENERButton_4.FontWeight = 'bold';
            app.DETENERButton_4.FontColor = [0.149 0.149 0.149];
            app.DETENERButton_4.Position = [449 337 106 31];
            app.DETENERButton_4.Text = 'DETENER';

            % Create AutoManualSwitch_3
            app.AutoManualSwitch_3 = uiswitch(app.ArticularTab, 'slider');
            app.AutoManualSwitch_3.Items = {'Automatico', 'Manual'};
            app.AutoManualSwitch_3.ValueChangedFcn = createCallbackFcn(app, @AutoManualSwitchValueChanged, true);
            app.AutoManualSwitch_3.FontSize = 14;
            app.AutoManualSwitch_3.Position = [735 316 60 27];
            app.AutoManualSwitch_3.Value = 'Automatico';

            % Create Led_manu
            app.Led_manu = uilamp(app.ArticularTab);
            app.Led_manu.Position = [857 316 25 25];

            % Create ESPACIOARTICULARLabel
            app.ESPACIOARTICULARLabel = uilabel(app.ArticularTab);
            app.ESPACIOARTICULARLabel.HorizontalAlignment = 'center';
            app.ESPACIOARTICULARLabel.FontName = 'C059';
            app.ESPACIOARTICULARLabel.FontSize = 34;
            app.ESPACIOARTICULARLabel.Position = [121 814 709 85];
            app.ESPACIOARTICULARLabel.Text = 'ESPACIO ARTICULAR';

            % Create J1Label_2
            app.J1Label_2 = uilabel(app.ArticularTab);
            app.J1Label_2.HorizontalAlignment = 'center';
            app.J1Label_2.FontSize = 14;
            app.J1Label_2.Position = [150 666 89 22];
            app.J1Label_2.Text = 'Articulacion 2';

            % Create J1Label_3
            app.J1Label_3 = uilabel(app.ArticularTab);
            app.J1Label_3.HorizontalAlignment = 'center';
            app.J1Label_3.FontSize = 14;
            app.J1Label_3.Position = [154 518 89 22];
            app.J1Label_3.Text = 'Articulacion 3';

            % Create J1Label_4
            app.J1Label_4 = uilabel(app.ArticularTab);
            app.J1Label_4.HorizontalAlignment = 'center';
            app.J1Label_4.FontSize = 14;
            app.J1Label_4.Position = [453 793 89 22];
            app.J1Label_4.Text = 'Articulacion 4';

            % Create J1Label_5
            app.J1Label_5 = uilabel(app.ArticularTab);
            app.J1Label_5.HorizontalAlignment = 'center';
            app.J1Label_5.FontSize = 14;
            app.J1Label_5.Position = [453 666 89 22];
            app.J1Label_5.Text = 'Articulacion 5';

            % Create J1Label_6
            app.J1Label_6 = uilabel(app.ArticularTab);
            app.J1Label_6.HorizontalAlignment = 'center';
            app.J1Label_6.FontSize = 14;
            app.J1Label_6.Position = [453 518 89 22];
            app.J1Label_6.Text = 'Articulacion 6';

            % Create J1Label_7
            app.J1Label_7 = uilabel(app.ArticularTab);
            app.J1Label_7.HorizontalAlignment = 'center';
            app.J1Label_7.FontSize = 14;
            app.J1Label_7.Position = [167 363 66 22];
            app.J1Label_7.Text = 'Velocidad';

            % Create SlidervalLabel_4
            app.SlidervalLabel_4 = uilabel(app.ArticularTab);
            app.SlidervalLabel_4.HorizontalAlignment = 'right';
            app.SlidervalLabel_4.Position = [110 283 32 22];
            app.SlidervalLabel_4.Text = 'Valor';

            % Create EditFieldSpeed
            app.EditFieldSpeed = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldSpeed.ValueChangedFcn = createCallbackFcn(app, @EditFieldSpeedValueChanged, true);
            app.EditFieldSpeed.Position = [157 283 100 22];

            % Create J1Label
            app.J1Label = uilabel(app.ArticularTab);
            app.J1Label.HorizontalAlignment = 'center';
            app.J1Label.FontSize = 14;
            app.J1Label.Position = [150 793 89 22];
            app.J1Label.Text = 'Articulacion 1';

            % Create SliderJoint1
            app.SliderJoint1 = uislider(app.ArticularTab);
            app.SliderJoint1.Limits = [-175 175];
            app.SliderJoint1.ValueChangingFcn = createCallbackFcn(app, @SliderJoint1ValueChanging, true);
            app.SliderJoint1.FontColor = [1 0.4118 0.1608];
            app.SliderJoint1.Position = [119 782 150 3];

            % Create ValorLabel
            app.ValorLabel = uilabel(app.ArticularTab);
            app.ValorLabel.HorizontalAlignment = 'right';
            app.ValorLabel.Position = [109 584 32 22];
            app.ValorLabel.Text = 'Valor';

            % Create EditFieldJoint2
            app.EditFieldJoint2 = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldJoint2.ValueChangedFcn = createCallbackFcn(app, @EditFieldJoint2ValueChanged, true);
            app.EditFieldJoint2.Position = [156 584 100 22];

            % Create ValorLabel_2
            app.ValorLabel_2 = uilabel(app.ArticularTab);
            app.ValorLabel_2.HorizontalAlignment = 'right';
            app.ValorLabel_2.Position = [109 437 32 22];
            app.ValorLabel_2.Text = 'Valor';

            % Create EditFieldJoint3
            app.EditFieldJoint3 = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldJoint3.ValueChangedFcn = createCallbackFcn(app, @EditFieldJoint3ValueChanged, true);
            app.EditFieldJoint3.Position = [156 437 100 22];

            % Create ValorLabel_3
            app.ValorLabel_3 = uilabel(app.ArticularTab);
            app.ValorLabel_3.HorizontalAlignment = 'right';
            app.ValorLabel_3.Position = [415 724 32 22];
            app.ValorLabel_3.Text = 'Valor';

            % Create EditFieldJoint4
            app.EditFieldJoint4 = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldJoint4.ValueChangedFcn = createCallbackFcn(app, @EditFieldJoint4ValueChanged, true);
            app.EditFieldJoint4.Position = [462 724 100 22];

            % Create ValorLabel_4
            app.ValorLabel_4 = uilabel(app.ArticularTab);
            app.ValorLabel_4.HorizontalAlignment = 'right';
            app.ValorLabel_4.Position = [417 437 32 22];
            app.ValorLabel_4.Text = 'Valor';

            % Create EditFieldJoint6
            app.EditFieldJoint6 = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldJoint6.ValueChangedFcn = createCallbackFcn(app, @EditFieldJoint6ValueChanged, true);
            app.EditFieldJoint6.Position = [464 437 100 22];

            % Create ValorLabel_5
            app.ValorLabel_5 = uilabel(app.ArticularTab);
            app.ValorLabel_5.HorizontalAlignment = 'right';
            app.ValorLabel_5.Position = [107 722 32 22];
            app.ValorLabel_5.Text = 'Valor';

            % Create EditFieldJoint1
            app.EditFieldJoint1 = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldJoint1.ValueChangedFcn = createCallbackFcn(app, @EditFieldJoint1ValueChanged, true);
            app.EditFieldJoint1.Position = [154 722 100 22];

            % Create ValorLabel_6
            app.ValorLabel_6 = uilabel(app.ArticularTab);
            app.ValorLabel_6.HorizontalAlignment = 'right';
            app.ValorLabel_6.Position = [418 584 32 22];
            app.ValorLabel_6.Text = 'Valor';

            % Create EditFieldJoint5
            app.EditFieldJoint5 = uieditfield(app.ArticularTab, 'numeric');
            app.EditFieldJoint5.ValueChangedFcn = createCallbackFcn(app, @EditFieldJoint5ValueChanged, true);
            app.EditFieldJoint5.Position = [465 584 100 22];

            % Create Button_CARGAR_GEMELO_DIGITAL
            app.Button_CARGAR_GEMELO_DIGITAL = uibutton(app.ArticularTab, 'push');
            app.Button_CARGAR_GEMELO_DIGITAL.ButtonPushedFcn = createCallbackFcn(app, @Button_CARGAR_GEMELO_DIGITALPushed, true);
            app.Button_CARGAR_GEMELO_DIGITAL.Icon = fullfile(pathToMLAPP, 'robotan5.png');
            app.Button_CARGAR_GEMELO_DIGITAL.Position = [614 546 275 262];
            app.Button_CARGAR_GEMELO_DIGITAL.Text = '';

            % Create CARGARGEMELODIGITALLabel_2
            app.CARGARGEMELODIGITALLabel_2 = uilabel(app.ArticularTab);
            app.CARGARGEMELODIGITALLabel_2.FontSize = 14;
            app.CARGARGEMELODIGITALLabel_2.FontWeight = 'bold';
            app.CARGARGEMELODIGITALLabel_2.Position = [652 564 216 42];
            app.CARGARGEMELODIGITALLabel_2.Text = 'CARGAR GEMELO DIGITAL';

            % Create RESETButton_2
            app.RESETButton_2 = uibutton(app.ArticularTab, 'push');
            app.RESETButton_2.ButtonPushedFcn = createCallbackFcn(app, @RESETButton_2Pushed, true);
            app.RESETButton_2.BackgroundColor = [1 0.4118 0.1608];
            app.RESETButton_2.FontWeight = 'bold';
            app.RESETButton_2.Position = [704 418 103 34];
            app.RESETButton_2.Text = 'RESET';

            % Create CartesianoTab
            app.CartesianoTab = uitab(app.TabGroup);
            app.CartesianoTab.SizeChangedFcn = createCallbackFcn(app, @CartesianoTabSizeChanged, true);
            app.CartesianoTab.Title = 'Cartesiano';
            app.CartesianoTab.BackgroundColor = 'none';

            % Create AADIRButton
            app.AADIRButton = uibutton(app.CartesianoTab, 'push');
            app.AADIRButton.ButtonPushedFcn = createCallbackFcn(app, @AADIRButtonPushed, true);
            app.AADIRButton.BackgroundColor = [1 1 1];
            app.AADIRButton.FontWeight = 'bold';
            app.AADIRButton.Position = [473 753 103 33];
            app.AADIRButton.Text = 'AÑADIR';

            % Create MOVERButton_2
            app.MOVERButton_2 = uibutton(app.CartesianoTab, 'push');
            app.MOVERButton_2.ButtonPushedFcn = createCallbackFcn(app, @MOVERButton_2Pushed, true);
            app.MOVERButton_2.BackgroundColor = [1 1 1];
            app.MOVERButton_2.FontWeight = 'bold';
            app.MOVERButton_2.Position = [473 659 106 34];
            app.MOVERButton_2.Text = 'MOVER';

            % Create XEditFieldLabel
            app.XEditFieldLabel = uilabel(app.CartesianoTab);
            app.XEditFieldLabel.HorizontalAlignment = 'right';
            app.XEditFieldLabel.FontSize = 14;
            app.XEditFieldLabel.FontWeight = 'bold';
            app.XEditFieldLabel.FontColor = [1 0.4118 0.1608];
            app.XEditFieldLabel.Position = [41 761 25 22];
            app.XEditFieldLabel.Text = 'X';

            % Create XEditField
            app.XEditField = uieditfield(app.CartesianoTab, 'numeric');
            app.XEditField.ValueChangedFcn = createCallbackFcn(app, @XEditFieldValueChanged, true);
            app.XEditField.Position = [81 761 100 22];

            % Create YEditFieldLabel
            app.YEditFieldLabel = uilabel(app.CartesianoTab);
            app.YEditFieldLabel.HorizontalAlignment = 'right';
            app.YEditFieldLabel.FontSize = 14;
            app.YEditFieldLabel.FontWeight = 'bold';
            app.YEditFieldLabel.FontColor = [0.851 0.3255 0.098];
            app.YEditFieldLabel.Position = [42 687 25 22];
            app.YEditFieldLabel.Text = 'Y';

            % Create YEditField
            app.YEditField = uieditfield(app.CartesianoTab, 'numeric');
            app.YEditField.ValueChangedFcn = createCallbackFcn(app, @YEditFieldValueChanged, true);
            app.YEditField.Position = [82 687 100 22];

            % Create ZEditFieldLabel
            app.ZEditFieldLabel = uilabel(app.CartesianoTab);
            app.ZEditFieldLabel.HorizontalAlignment = 'right';
            app.ZEditFieldLabel.FontSize = 14;
            app.ZEditFieldLabel.FontWeight = 'bold';
            app.ZEditFieldLabel.FontColor = [1 0.4118 0.1608];
            app.ZEditFieldLabel.Position = [43 609 25 22];
            app.ZEditFieldLabel.Text = 'Z';

            % Create ZEditField
            app.ZEditField = uieditfield(app.CartesianoTab, 'numeric');
            app.ZEditField.ValueChangedFcn = createCallbackFcn(app, @ZEditFieldValueChanged, true);
            app.ZEditField.Position = [83 609 100 22];

            % Create RxEditFieldLabel
            app.RxEditFieldLabel = uilabel(app.CartesianoTab);
            app.RxEditFieldLabel.HorizontalAlignment = 'right';
            app.RxEditFieldLabel.FontSize = 14;
            app.RxEditFieldLabel.FontWeight = 'bold';
            app.RxEditFieldLabel.FontColor = [1 0.4118 0.1608];
            app.RxEditFieldLabel.Position = [242 761 25 22];
            app.RxEditFieldLabel.Text = 'Rx';

            % Create RxEditField
            app.RxEditField = uieditfield(app.CartesianoTab, 'numeric');
            app.RxEditField.ValueChangedFcn = createCallbackFcn(app, @RxEditFieldValueChanged, true);
            app.RxEditField.Position = [282 761 100 22];

            % Create RyEditFieldLabel
            app.RyEditFieldLabel = uilabel(app.CartesianoTab);
            app.RyEditFieldLabel.HorizontalAlignment = 'right';
            app.RyEditFieldLabel.FontSize = 14;
            app.RyEditFieldLabel.FontWeight = 'bold';
            app.RyEditFieldLabel.FontColor = [1 0.4118 0.1608];
            app.RyEditFieldLabel.Position = [242 687 25 22];
            app.RyEditFieldLabel.Text = 'Ry';

            % Create RyEditField
            app.RyEditField = uieditfield(app.CartesianoTab, 'numeric');
            app.RyEditField.ValueChangedFcn = createCallbackFcn(app, @RyEditFieldValueChanged, true);
            app.RyEditField.Position = [282 687 100 22];

            % Create RzEditFieldLabel
            app.RzEditFieldLabel = uilabel(app.CartesianoTab);
            app.RzEditFieldLabel.HorizontalAlignment = 'right';
            app.RzEditFieldLabel.FontSize = 14;
            app.RzEditFieldLabel.FontWeight = 'bold';
            app.RzEditFieldLabel.FontColor = [1 0.4118 0.1608];
            app.RzEditFieldLabel.Position = [242 609 25 22];
            app.RzEditFieldLabel.Text = 'Rz';

            % Create RzEditField
            app.RzEditField = uieditfield(app.CartesianoTab, 'numeric');
            app.RzEditField.ValueChangedFcn = createCallbackFcn(app, @RzEditFieldValueChanged, true);
            app.RzEditField.Position = [282 609 100 22];

            % Create PosicionesCartesianasTextAreaLabel
            app.PosicionesCartesianasTextAreaLabel = uilabel(app.CartesianoTab);
            app.PosicionesCartesianasTextAreaLabel.HorizontalAlignment = 'right';
            app.PosicionesCartesianasTextAreaLabel.FontName = 'C059';
            app.PosicionesCartesianasTextAreaLabel.FontSize = 24;
            app.PosicionesCartesianasTextAreaLabel.Position = [180 463 263 30];
            app.PosicionesCartesianasTextAreaLabel.Text = 'Posiciones Cartesianas';

            % Create PosicionesCartesianasTextArea
            app.PosicionesCartesianasTextArea = uitextarea(app.CartesianoTab);
            app.PosicionesCartesianasTextArea.ValueChangedFcn = createCallbackFcn(app, @PosicionesCartesianasTextAreaValueChanged, true);
            app.PosicionesCartesianasTextArea.FontSize = 14;
            app.PosicionesCartesianasTextArea.Position = [46 165 519 281];

            % Create AutoManualSwitch_2
            app.AutoManualSwitch_2 = uiswitch(app.CartesianoTab, 'slider');
            app.AutoManualSwitch_2.Items = {'Automatico', 'Manual'};
            app.AutoManualSwitch_2.ValueChangedFcn = createCallbackFcn(app, @AutoManualSwitch_2ValueChanged, true);
            app.AutoManualSwitch_2.FontSize = 14;
            app.AutoManualSwitch_2.Position = [724 271 56 25];
            app.AutoManualSwitch_2.Value = 'Manual';

            % Create Led_Manual
            app.Led_Manual = uilamp(app.CartesianoTab);
            app.Led_Manual.Position = [847 271 25 25];

            % Create SliderSpeed_2
            app.SliderSpeed_2 = uislider(app.CartesianoTab);
            app.SliderSpeed_2.ValueChangedFcn = createCallbackFcn(app, @SliderSpeed_2ValueChanged, true);
            app.SliderSpeed_2.FontColor = [1 0.4118 0.1608];
            app.SliderSpeed_2.Position = [666 415 155 3];

            % Create VelocidadLabel
            app.VelocidadLabel = uilabel(app.CartesianoTab);
            app.VelocidadLabel.HorizontalAlignment = 'right';
            app.VelocidadLabel.FontSize = 14;
            app.VelocidadLabel.Position = [703 428 66 22];
            app.VelocidadLabel.Text = 'Velocidad';

            % Create SlidervalLabel_5
            app.SlidervalLabel_5 = uilabel(app.CartesianoTab);
            app.SlidervalLabel_5.HorizontalAlignment = 'right';
            app.SlidervalLabel_5.Position = [662 341 32 22];
            app.SlidervalLabel_5.Text = 'Valor';

            % Create EditFieldSpeed_2
            app.EditFieldSpeed_2 = uieditfield(app.CartesianoTab, 'numeric');
            app.EditFieldSpeed_2.ValueChangedFcn = createCallbackFcn(app, @EditFieldSpeed_2ValueChanged, true);
            app.EditFieldSpeed_2.Position = [709 341 100 22];

            % Create DETENERButton_2
            app.DETENERButton_2 = uibutton(app.CartesianoTab, 'push');
            app.DETENERButton_2.ButtonPushedFcn = createCallbackFcn(app, @DETENERButton_2Pushed, true);
            app.DETENERButton_2.BackgroundColor = [1 0.4118 0.1608];
            app.DETENERButton_2.FontWeight = 'bold';
            app.DETENERButton_2.Position = [473 609 103 34];
            app.DETENERButton_2.Text = 'DETENER';

            % Create ESPACIOCARTESIANOLabel
            app.ESPACIOCARTESIANOLabel = uilabel(app.CartesianoTab);
            app.ESPACIOCARTESIANOLabel.HorizontalAlignment = 'center';
            app.ESPACIOCARTESIANOLabel.FontName = 'C059';
            app.ESPACIOCARTESIANOLabel.FontSize = 34;
            app.ESPACIOCARTESIANOLabel.Position = [121 814 709 85];
            app.ESPACIOCARTESIANOLabel.Text = 'ESPACIO CARTESIANO';

            % Create Button_CARGAR_GEMELO_DIGITAL2
            app.Button_CARGAR_GEMELO_DIGITAL2 = uibutton(app.CartesianoTab, 'push');
            app.Button_CARGAR_GEMELO_DIGITAL2.ButtonPushedFcn = createCallbackFcn(app, @Button_CARGAR_GEMELO_DIGITAL2Pushed, true);
            app.Button_CARGAR_GEMELO_DIGITAL2.Icon = fullfile(pathToMLAPP, 'robotan5.png');
            app.Button_CARGAR_GEMELO_DIGITAL2.Position = [627 539 275 262];
            app.Button_CARGAR_GEMELO_DIGITAL2.Text = '';

            % Create CARGARGEMELODIGITALLabel
            app.CARGARGEMELODIGITALLabel = uilabel(app.CartesianoTab);
            app.CARGARGEMELODIGITALLabel.FontSize = 14;
            app.CARGARGEMELODIGITALLabel.FontWeight = 'bold';
            app.CARGARGEMELODIGITALLabel.Position = [673 553 216 42];
            app.CARGARGEMELODIGITALLabel.Text = 'CARGAR GEMELO DIGITAL';

            % Create x_Lim830320Label
            app.x_Lim830320Label = uilabel(app.CartesianoTab);
            app.x_Lim830320Label.FontWeight = 'bold';
            app.x_Lim830320Label.Position = [81 730 116 22];
            app.x_Lim830320Label.Text = 'x_Lim = [-830, -320]';

            % Create y_Lim500500Label
            app.y_Lim500500Label = uilabel(app.CartesianoTab);
            app.y_Lim500500Label.FontWeight = 'bold';
            app.y_Lim500500Label.Position = [78 659 112 22];
            app.y_Lim500500Label.Text = 'y_Lim = [-500, 500]';

            % Create z_Lim0720Label
            app.z_Lim0720Label = uilabel(app.CartesianoTab);
            app.z_Lim0720Label.FontWeight = 'bold';
            app.z_Lim0720Label.Position = [85 584 94 22];
            app.z_Lim0720Label.Text = 'z_Lim = [0, 720]';

            % Create CLEARButton
            app.CLEARButton = uibutton(app.CartesianoTab, 'push');
            app.CLEARButton.ButtonPushedFcn = createCallbackFcn(app, @CLEARButtonPushed, true);
            app.CLEARButton.BackgroundColor = [1 1 1];
            app.CLEARButton.FontWeight = 'bold';
            app.CLEARButton.Position = [474 708 103 33];
            app.CLEARButton.Text = 'CLEAR';

            % Create RESETButton
            app.RESETButton = uibutton(app.CartesianoTab, 'push');
            app.RESETButton.ButtonPushedFcn = createCallbackFcn(app, @RESETButtonPushed, true);
            app.RESETButton.BackgroundColor = [1 0.4118 0.1608];
            app.RESETButton.FontWeight = 'bold';
            app.RESETButton.Position = [692 165 103 34];
            app.RESETButton.Text = 'RESET';

            % Create SimulinkTab
            app.SimulinkTab = uitab(app.TabGroup);
            app.SimulinkTab.Title = 'Simulink';
            app.SimulinkTab.BackgroundColor = 'none';

            % Create CARGARTXTButton
            app.CARGARTXTButton = uibutton(app.SimulinkTab, 'push');
            app.CARGARTXTButton.ButtonPushedFcn = createCallbackFcn(app, @CARGARTXTButtonPushed, true);
            app.CARGARTXTButton.FontWeight = 'bold';
            app.CARGARTXTButton.Position = [425 461 135 53];
            app.CARGARTXTButton.Text = 'CARGAR TXT';

            % Create SimulationControls
            app.SimulationControls = uisimcontrols(app.SimulinkTab);
            app.SimulationControls.Simulation = app.Simulation;
            app.SimulationControls.Position = [121 393 207 93];

            % Create MOVERButton_3
            app.MOVERButton_3 = uibutton(app.SimulinkTab, 'push');
            app.MOVERButton_3.ButtonPushedFcn = createCallbackFcn(app, @MOVERButton_3Pushed, true);
            app.MOVERButton_3.FontWeight = 'bold';
            app.MOVERButton_3.Position = [686 354 137 53];
            app.MOVERButton_3.Text = 'MOVER';

            % Create DETENERButton_3
            app.DETENERButton_3 = uibutton(app.SimulinkTab, 'push');
            app.DETENERButton_3.ButtonPushedFcn = createCallbackFcn(app, @DETENERButton_3Pushed, true);
            app.DETENERButton_3.BackgroundColor = [1 0.4118 0.1608];
            app.DETENERButton_3.FontWeight = 'bold';
            app.DETENERButton_3.Position = [686 462 125 53];
            app.DETENERButton_3.Text = 'DETENER';

            % Create ESPACIODESIMULACIONLabel
            app.ESPACIODESIMULACIONLabel = uilabel(app.SimulinkTab);
            app.ESPACIODESIMULACIONLabel.HorizontalAlignment = 'center';
            app.ESPACIODESIMULACIONLabel.FontName = 'C059';
            app.ESPACIODESIMULACIONLabel.FontSize = 34;
            app.ESPACIODESIMULACIONLabel.Position = [106 823 709 85];
            app.ESPACIODESIMULACIONLabel.Text = 'ESPACIO DE SIMULACION';

            % Create Button_4
            app.Button_4 = uibutton(app.SimulinkTab, 'push');
            app.Button_4.Icon = fullfile(pathToMLAPP, 'robotan5.png');
            app.Button_4.FontColor = [1 1 1];
            app.Button_4.Position = [315 553 289 282];
            app.Button_4.Text = '';

            % Create RESETButton_3
            app.RESETButton_3 = uibutton(app.SimulinkTab, 'push');
            app.RESETButton_3.ButtonPushedFcn = createCallbackFcn(app, @RESETButton_3Pushed, true);
            app.RESETButton_3.BackgroundColor = [0.651 0.651 0.651];
            app.RESETButton_3.FontWeight = 'bold';
            app.RESETButton_3.Position = [424 354 131 53];
            app.RESETButton_3.Text = 'RESET';

            % Create SIMULARLabel
            app.SIMULARLabel = uilabel(app.SimulinkTab);
            app.SIMULARLabel.FontName = 'C059';
            app.SIMULARLabel.FontSize = 18;
            app.SIMULARLabel.FontWeight = 'bold';
            app.SIMULARLabel.Position = [169 485 220 33];
            app.SIMULARLabel.Text = 'SIMULAR';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Interfaz_App_Designer

            % Associate the Simulink Model
            app.Simulation = simulation('simulador_trayectorias_AN5');

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            close all;
            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end