% Declaración de variables globales
global pub_inverse_kinematics_result pub_api_command current_joint_position ...
       target_joint_position is_waiting_for_position processing_commands myApp;

% Inicializar variables globales
current_joint_position    = zeros(6, 1);  % Posición actual del robot (J1 a J6)
target_joint_position     = zeros(6, 1);  % Posición objetivo a alcanzar
is_waiting_for_position   = false;        % Bandera para indicar si se está esperando la posición
processing_commands       = false;        % Bandera para indicar si se están procesando comandos

% Inicializa el nodo ROS2
node = ros2node("/matlab_ik_node");

% Suscribirse al tópico de posición cartesiana
sub_cartesian_position = ros2subscriber(node, "/input_cartesian_position", "std_msgs/String", @poseCallback);

% Suscribirse al tópico del archivo de posiciones
sub_cartesian_path = ros2subscriber(node, "/input_cartesian_path", "std_msgs/String", @filePathCallback);

% Suscribirse al tópico de posiciones conjuntas
sub_joint_position = ros2subscriber(node, "/current_joint_position", "std_msgs/String", @jointPositionCallback);

% Inicializa un publicador para enviar los comandos API
pub_api_command = ros2publisher(node, "/api_command", "std_msgs/String", 'Durability', 'transientlocal');

% Inicializa un publicador para los resultados de cinemática inversa
pub_inverse_kinematics_result = ros2publisher(node, "/output_joint_position", "std_msgs/String");

% Mostrar mensaje de espera
logMessage('Esperando mensajes de ROS2...');

%% Función auxiliar para registrar mensajes en consola y en el TextArea de App Designer
function logMessage(msg)
    global myApp;
    % Si la app está abierta y es válida, usar el método de registro
    if ~isempty(myApp) && isvalid(myApp)
        myApp.logInTextArea(msg);
    end
    disp(msg);
end

%% --- Callbacks ---

% Callback para procesar los valores cartesianos recibidos
function poseCallback(msg)
    global pub_inverse_kinematics_result;

    data_str = msg.data;
    values = str2double(strsplit(data_str, ','));

    if length(values) ~= 6
        logMessage('Datos de pose inválidos recibidos.');
        return;
    end

    x  = values(1); y  = values(2); z  = values(3);
    rx = values(4); ry = values(5); rz = values(6);

    % Calcular la cinemática inversa (se asume que la función fr5_ik existe)
    hi = fr5_ik(x, y, z, rx, ry, rz);

    if isempty(hi) || length(hi) ~= 6
        logMessage('Error en la cinemática inversa.');
        return;
    end

    % Formatear los resultados con 17 decimales
    hi_str = sprintf('%.17f,%.17f,%.17f,%.17f,%.17f,%.17f', hi);

    % Enviar los resultados al tópico correspondiente
    result_msg = ros2message("std_msgs/String");
    result_msg.data = hi_str;
    send(pub_inverse_kinematics_result, result_msg);

    logMessage(['Resultados de la cinemática inversa: ' hi_str]);
end

% Callback para procesar el nombre del archivo recibido
function filePathCallback(msg)
    global processing_commands;

    if processing_commands
        logMessage('Ya se están procesando comandos. Ignorando nuevo archivo.');
        return;
    end

    file_name = msg.data;
    validate_and_convert_positions(file_name);
end

% Callback para actualizar la posición conjunta actual
function jointPositionCallback(msg)
    global current_joint_position target_joint_position is_waiting_for_position;

    data_str = msg.data;
    values = str2double(strsplit(data_str, ','));

    if length(values) == 6
        current_joint_position = values;

        if is_waiting_for_position
            % Comparar la posición actual con la posición objetivo con tolerancia de 0.5 grados
            if all(abs(current_joint_position - target_joint_position) <= 0.5)
                logMessage('El robot ha alcanzado la posición objetivo.');
                is_waiting_for_position = false;
            end
        end
    else
        logMessage('Recibió datos inválidos de posiciones conjuntas.');
    end
end

%% Función para validar y convertir posiciones del archivo
function validate_and_convert_positions(file_name)
    global pub_api_command current_joint_position target_joint_position is_waiting_for_position processing_commands;

    processing_commands = true;

    %% 1. Leer la primera línea del archivo (encabezado)
    fid = fopen(file_name, 'r');
    if fid == -1
        error('No se pudo leer el archivo. Verifica el nombre y el formato.');
    end
    headerLine = strtrim(fgetl(fid));  % Leer y eliminar espacios en blanco
    fclose(fid);

    %% 2. Leer el resto de los datos (omitiendo la primera línea)
    try
        dataStruct = importdata(file_name, ',', 1);
        data = dataStruct.data;
    catch
        error('Error al leer los datos numéricos del archivo.');
    end

    [num_rows, num_cols] = size(data);
    if num_cols < 8
        error('El archivo debe contener 8 columnas: x, y, z, rx, ry, rz, velocidad y control.');
    end

    %% 3. Validación de datos
    % Límites definidos para posiciones y orientaciones
    x_lim    = [-830, -320];
    y_lim    = [-500, 500];
    z_lim    = [0, 720];
    rx_lim_1 = [-180, -20];
    rx_lim_2 = [20, 180];
    j4_lim   = [-267, 90];
    j5_lim   = [-90, 90];

    % Determinar si todos los controles son cero (para SplineStart/SplineEnd)
    all_control_zero = all(data(:, 8) == 0);

    all_positions_valid = true;
    valid_positions = [];

    for i = 1:num_rows
        x  = data(i, 1);  y  = data(i, 2);  z  = data(i, 3);
        rx = data(i, 4);  ry = data(i, 5);  rz = data(i, 6);
        velocidad = data(i, 7);  control = data(i, 8);

        % Validar límites de posición
        if ~(x >= x_lim(1) && x <= x_lim(2) && ...
             y >= y_lim(1) && y <= y_lim(2) && ...
             z >= z_lim(1) && z <= z_lim(2))
            logMessage(['Error: Posición fuera de límites en fila ' num2str(i) '.']);
            all_positions_valid = false;
            break;
        end

        % Validar límites de orientación (rx)
        if ~((rx >= rx_lim_1(1) && rx <= rx_lim_1(2)) || (rx >= rx_lim_2(1) && rx <= rx_lim_2(2)))
            logMessage(['Error: Valor de rx fuera de límites en fila ' num2str(i) '.']);
            all_positions_valid = false;
            break;
        end

        % Calcular la conversión a posiciones articulares usando cinemática inversa
        qsalida = fr5_ik(x, y, z, rx, ry, rz);
        if isempty(qsalida) || length(qsalida) ~= 6
            logMessage(['Error en la cinemática inversa en fila ' num2str(i) '.']);
            all_positions_valid = false;
            break;
        end

        j4 = qsalida(4); j5 = qsalida(5);
        % Validar límites para las articulaciones J4 y J5
        if ~((j4 >= 0 && j4 <= 90 && j5 >= j5_lim(1) && j5 <= j5_lim(2)) || ...
             (j4 >= -180 && j4 <= 0) || ...
             (j4 >= -267 && j4 <= -180 && j5 >= j5_lim(1) && j5 <= j5_lim(2)))
            logMessage(['Error: Valores de J4 o J5 fuera de límites en fila ' num2str(i) '.']);
            all_positions_valid = false;
            break;
        end

        % Almacenar la fila validada (manteniendo las 8 columnas originales)
        valid_positions = [valid_positions; x, y, z, rx, ry, rz, velocidad, control];
    end

    % Si alguna validación falla, se termina la función
    if ~all_positions_valid
        logMessage('Error: Una o más posiciones no cumplen con los límites. No se enviará ningún comando de movimiento.');
        processing_commands = false;
        return;
    end

    %% 4. Ejecución según el encabezado
    if strcmpi(headerLine, 'cartesiano')
        % ---------------------------
        % Rama "cartesiano"
        % ---------------------------

        % (A) Copiar el archivo (desde la segunda fila) en python_position.txt
        fid_in = fopen(file_name, 'r');
        if fid_in == -1
            error('No se pudo abrir el archivo original para copiar.');
        end
        fid_out = fopen('python_position.txt', 'w');
        if fid_out == -1
            fclose(fid_in);
            error('No se pudo crear el archivo python_position.txt.');
        end
        % Leer y descartar la primera línea (encabezado)
        fgetl(fid_in);
        % Copiar el resto de las líneas
        while true
            tline = fgetl(fid_in);
            if ~ischar(tline)
                break;
            end
            fprintf(fid_out, '%s\n', tline);
        end
        fclose(fid_in);
        fclose(fid_out);
        logMessage('Se ha creado el archivo python_position.txt con los datos desde la segunda fila.');

        % (B) Crear el archivo joint_python_position.txt con la conversión a posiciones articulares
        fid_joint = fopen('joint_python_position.txt', 'w');
        if fid_joint == -1
            error('No se pudo crear el archivo joint_python_position.txt.');
        end
        for i = 1:num_rows
            x  = data(i, 1);  y  = data(i, 2);  z  = data(i, 3);
            rx = data(i, 4);  ry = data(i, 5);  rz = data(i, 6);
            velocidad = data(i, 7);  control = data(i, 8);

            joint_vals = fr5_ik(x, y, z, rx, ry, rz);
            fprintf(fid_joint, '%.17f,%.17f,%.17f,%.17f,%.17f,%.17f,%.2f,%.2f\n', ...
                joint_vals(1), joint_vals(2), joint_vals(3), ...
                joint_vals(4), joint_vals(5), joint_vals(6), ...
                velocidad, control);
        end
        fclose(fid_joint);
        logMessage('Se ha creado el archivo joint_python_position.txt con la conversión a valores articulares y las columnas 7 y 8 originales.');

        % (C) Ejecutar el script Python
        scriptPath = "/home/tarw/ros2_ws/src/code/code/MoveL.py";
        cmd = "bash -c 'export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6; " + ...
              "source /opt/ros/humble/setup.bash; " + ...
              "source /home/tarw/ros2_ws/install/setup.bash; " + ...
              "python3 " + scriptPath + "'";
        [status, cmdout] = system(cmd);
        if status ~= 0
            logMessage("Ocurrió un error al ejecutar el script de Python:");
            logMessage(cmdout);
        else
            logMessage("Script de Python se ejecutó exitosamente:");
            logMessage(cmdout);
        end

    elseif strcmpi(headerLine, 'articular')
        % ---------------------------
        % Rama "articular"
        % ---------------------------
        logMessage('Todas las posiciones están dentro de los límites y serán convertidas.');

        % Enviar comando SplineStart() si todos los controles son cero
        if all(valid_positions(:,8) == 0)
            msg = ros2message(pub_api_command);
            msg.data = 'SplineStart()';
            send(pub_api_command, msg);
            pause(0.05);
        end

        % Envío de comandos en lotes
        max_index = 5;
        batch_size = max_index;
        num_batches = ceil(size(valid_positions, 1) / batch_size);
        configs_matrix = zeros(size(valid_positions, 1), 6);

        for batch = 1:num_batches
            start_idx = (batch - 1) * batch_size + 1;
            end_idx = min(batch * batch_size, size(valid_positions, 1));

            % Enviar los comandos JNTPoint para cada punto del lote
            for i = start_idx:end_idx
                index = mod(i-1, max_index) + 1;
                x  = valid_positions(i, 1);  y  = valid_positions(i, 2);  z  = valid_positions(i, 3);
                rx = valid_positions(i, 4);  ry = valid_positions(i, 5);  rz = valid_positions(i, 6);
                velocidad = valid_positions(i, 7); control = valid_positions(i, 8);
                qsalida = fr5_ik(x, y, z, rx, ry, rz);
                configs_matrix(i, :) = qsalida;
                cmd_str = sprintf('JNTPoint(%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)', ...
                    index, configs_matrix(i,1), configs_matrix(i,2), configs_matrix(i,3), ...
                    configs_matrix(i,4), configs_matrix(i,5), configs_matrix(i,6));
                msg = ros2message(pub_api_command);
                msg.data = cmd_str;
                send(pub_api_command, msg);
                pause(0.01);
            end

            % Enviar comandos de movimiento basados en el control
            for i = start_idx:end_idx
                index = mod(i-1, max_index) + 1;
                velocidad = valid_positions(i,7);
                control = valid_positions(i,8);
                if control ~= 0
                    cmd_str = sprintf('MoveJ(JNT%d,%.2f)', index, velocidad);
                    msg = ros2message(pub_api_command);
                    msg.data = cmd_str;
                    send(pub_api_command, msg);

                    target_joint_position = configs_matrix(i, :);
                    is_waiting_for_position = true;
                    waitForRobotToReachPosition(target_joint_position, 0.5);

                    logMessage(['Esperando ' num2str(control) ' segundos antes del siguiente comando.']);
                    pause(control);
                else
                    cmd_str = sprintf('SplinePTP(JNT%d,%.2f)', index, velocidad);
                    msg = ros2message(pub_api_command);
                    msg.data = cmd_str;
                    send(pub_api_command, msg);
                    pause(0.05);
                end
            end

            pause(0.05);
        end

        % Enviar comando SplineEnd() si todos los controles son cero
        if all(valid_positions(:,8) == 0)
            msg = ros2message(pub_api_command);
            msg.data = 'SplineEnd()';
            send(pub_api_command, msg);
        end

        logMessage('Matriz de configuraciones articulares (en grados):');
        logMessage(mat2str(configs_matrix));

    else
        logMessage('Encabezado no reconocido. Debe ser "articular" o "cartesiano".');
    end

    % Reiniciar la bandera de procesamiento
    processing_commands = false;
end

%% --- Función para Esperar a que el Robot Alcance la Posición Objetivo ---
function waitForRobotToReachPosition(target_position, tolerance)
    global current_joint_position target_joint_position is_waiting_for_position;

    target_joint_position = target_position;
    is_waiting_for_position = true;

    % Esperar hasta que se cumpla la condición (con pausas de 0.1 segundos)
    while is_waiting_for_position
        pause(0.1);
    end
end



