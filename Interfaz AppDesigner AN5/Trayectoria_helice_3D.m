% Parámetros de la espiral en 3D
radio_inicial = 20;          % Radio inicial de la espiral en mm
radio_final   = 100;         % Radio máximo de la espiral en mm
n_puntos    = 200;           % Número total de puntos en la espiral
centro      = [-497, -102, 377];  % Centro de la espiral (en mm)
vueltas     = 5;             % Número de vueltas de la espiral

% Parámetros de tiempo
Tfinal            = 4;       % Tiempo total de simulación (en segundos)
velocidad_definida = 80;     % Velocidad constante usada en la última columna
Tem               = Tfinal / n_puntos;   % Tiempo de muestreo
instante          = linspace(0, Tfinal, n_puntos)';  % Vector de tiempos

% Crear una lista de ángulos igualmente espaciados
angulos = linspace(0, vueltas * 2 * pi, n_puntos);

% Crear una matriz para almacenar las posiciones cartesianas deseadas
positions = zeros(n_puntos, 3);

% Calcular el radio de la espiral en cada punto
radios = linspace(radio_inicial, radio_final, n_puntos);

% Calcular las posiciones de la espiral en 3D
for i = 1:n_puntos
    x = centro(1) + radios(i) * cos(angulos(i));
    y = centro(2) + radios(i) * sin(angulos(i));
    z = centro(3) + (i / n_puntos) * (radio_final - radio_inicial);
    positions(i, :) = [x, y, z];
end

% Asignar posiciones calculadas a variables
cons1 = positions(:, 1);
cons2 = positions(:, 2);
cons3 = positions(:, 3);

% Definir rotaciones fijas
rotations = repmat([180, 0, 0], n_puntos, 1);
cons4 = rotations(:, 1);
cons5 = rotations(:, 2);
cons6 = rotations(:, 3);

% Guardar las posiciones y rotaciones en un archivo de texto
archivo_salida_txt = 'trayectoria_helice_3D.txt';
fileID = fopen(archivo_salida_txt, 'w');

% Escribir la palabra 'articular' en la primera fila del archivo
fprintf(fileID, 'articular\n');

% Escribir los datos de la trayectoria
for i = 1:n_puntos
    fprintf(fileID, '%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.3f\n', ...
        cons1(i), cons2(i), cons3(i), cons4(i), cons5(i), cons6(i), velocidad_definida, 0);
end
fclose(fileID);
disp(['Archivo guardado: ', archivo_salida_txt]);

% Graficar la trayectoria en 3D
figure;
plot3(cons1, cons2, cons3, 'o-');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Trayectoria en Hélice 3D');
grid on;
axis equal;

disp('Trayectoria en espiral generada y guardada correctamente.');
