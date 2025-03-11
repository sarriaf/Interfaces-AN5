radio = 120;          % Radio del círculo en mm
n_puntos = 60;        % Número total de puntos en el círculo
centro = [-497, -102, 377];  % Centro del círculo (en mm)
eje = 'xz';           % Eje en el que se dibujará el círculo ('xy', 'xz', 'yz')

% Parámetro de delay
delay = 0; % Valor inicial del delay (puedes cambiar este valor según lo necesites)

% Parámetros de tiempo
Tfinal = 2;  % Tiempo total de simulación (en segundos)

% Definir la velocidad para cada punto
velocidad_definida = 15;  % Velocidad constante que se usará en la última columna

% Calcular el tiempo de muestreo (Tem) basado en el número de puntos
Tem = Tfinal / n_puntos;

% Definir el tiempo de cada muestra
instante = linspace(0, Tfinal, n_puntos)'; % Vector de tiempos en que se aplicarán los puntos

% Crear una lista de ángulos igualmente espaciados para los puntos del círculo
angulos = linspace(0, 2*pi, n_puntos);

% Crear una matriz para almacenar las posiciones cartesianas deseadas
positions = zeros(n_puntos, 3);

% Calcular las posiciones del círculo basado en el eje seleccionado
for i = 1:n_puntos
    switch eje
        case 'xy'
            x = centro(1) + radio * cos(angulos(i));
            y = centro(2) + radio * sin(angulos(i));
            z = centro(3);  % Mantener z constante
        case 'xz'
            x = centro(1) + radio * cos(angulos(i));
            y = centro(2);  % Mantener y constante
            z = centro(3) + radio * sin(angulos(i));
        case 'yz'
            x = centro(1);  % Mantener x constante
            y = centro(2) + radio * cos(angulos(i));
            z = centro(3) + radio * sin(angulos(i));
    end
    positions(i, :) = [x, y, z];
end

% Asignar posiciones calculadas a las variables cons1, cons2, cons3
cons1 = positions(:, 1);
cons2 = positions(:, 2);
cons3 = positions(:, 3);

% Verificar que 'instante', 'cons1', 'cons2', y 'cons3' tengan el mismo tamaño
if length(instante) ~= length(cons1) || length(cons1) ~= length(cons2) || length(cons2) ~= length(cons3)
    error('Las dimensiones de instante, cons1, cons2 y cons3 no coinciden.');
end

% Rotaciones específicas según el eje
switch eje
    case 'xy'
        rotations = repmat([180, 0, 0], n_puntos, 1); % Roll = 180, Pitch = 0, Yaw = 0
    case 'xz'
        rotations = repmat([90.497, 65.669, -0.174], n_puntos, 1); % Roll, Pitch, Yaw específicos
    case 'yz'
        rotations = repmat([100, 65.669, -80], n_puntos, 1); % Roll, Pitch, Yaw específicos
end

% Asignar las rotaciones a las variables cons4, cons5, cons6
cons4 = rotations(:, 1);   % Roll en grados
cons5 = rotations(:, 2);   % Pitch en grados
cons6 = rotations(:, 3);   % Yaw en grados

% Crear la estructura que contendrá los datos para Simulink
variables_simulink = struct('instante', instante, 'cons1', cons1, 'cons2', cons2, 'cons3', cons3, ...
                            'cons4', cons4, 'cons5', cons5, 'cons6', cons6, 'Tfinal', Tfinal, ...
                            'Tem', Tem, 'n_puntos', n_puntos, 'radio', radio, 'centro', centro, 'eje', eje);

% Crear la estructura que contendrá las rotaciones
rotaciones_simulink = struct('rot_x', cons4, 'rot_y', cons5, 'rot_z', cons6);

% Guardar todas las variables relevantes en un archivo .mat
nombre_archivo_mat = 'posiciones_simulink.mat';
save(nombre_archivo_mat, 'variables_simulink', 'rotaciones_simulink', 'instante', 'cons1', 'cons2', 'cons3', 'cons4', 'cons5', 'cons6', 'centro', 'radio', 'eje', 'Tfinal', 'Tem', 'positions', 'angulos');
disp(['Variables guardadas en el archivo: ', nombre_archivo_mat]);

% Guardar las posiciones, rotaciones y los instantes en un archivo de texto
archivo_salida_txt = 'trayectoria_circulo.txt';
fileID = fopen(archivo_salida_txt, 'w');

% Escribir la primera línea con la palabra 'articular'
fprintf(fileID, 'articular\n');

% Guardar las posiciones, rotaciones, velocidad y delay en el archivo
for i = 1:n_puntos
    x = cons1(i);       % Posición X
    y = cons2(i);       % Posición Y
    z = cons3(i);       % Posición Z
    
    roll = cons4(i);    % Roll en grados
    pitch = cons5(i);   % Pitch en grados
    yaw = cons6(i);     % Yaw en grados
    
    % Escribir las posiciones, rotaciones, velocidad y delay en el archivo
    fprintf(fileID, '%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.3f\n', x, y, z, roll, pitch, yaw, velocidad_definida, delay);
end

% Cerrar el archivo de texto
fclose(fileID);
disp(['El archivo de texto con las posiciones y la velocidad ha sido guardado correctamente en: ', archivo_salida_txt]);
disp('Las variables también han sido guardadas en el workspace para Simulink.');

% Graficar la trayectoria en 3D
figure;
switch eje
    case 'xy'
        plot3(cons1, cons2, cons3, 'o-');
        xlabel('X (mm)');
        ylabel('Y (mm)');
        zlabel('Z (mm)');
        title('Trayectoria en el plano XY');
    case 'xz'
        plot3(cons1, cons2, cons3, 'o-');
        xlabel('X (mm)');
        ylabel('Y (mm)');
        zlabel('Z (mm)');
        title('Trayectoria en el plano XZ');
    case 'yz'
        plot3(cons1, cons2, cons3, 'o-');
        xlabel('X (mm)');
        ylabel('Y (mm)');
        zlabel('Z (mm)');
        title('Trayectoria en el plano YZ');
end
grid on;
axis equal;

% Guardar las variables en el espacio de trabajo para Simulink
assignin('base', 'variables_simulink', variables_simulink);
assignin('base', 'rotaciones_simulink', rotaciones_simulink);

disp('Las variables han sido guardadas en el workspace para Simulink.');
