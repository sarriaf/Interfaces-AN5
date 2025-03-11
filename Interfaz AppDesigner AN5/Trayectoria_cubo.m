% Parámetros del cubo
lado = 150;            % Longitud de cada lado del cubo en mm
centro = [-497, -102, 377];  % Centro del cubo (en mm)

% Definir los ocho vértices del cubo
vertices = [centro(1)-lado/2, centro(2)-lado/2, centro(3)-lado/2;
            centro(1)+lado/2, centro(2)-lado/2, centro(3)-lado/2;
            centro(1)+lado/2, centro(2)+lado/2, centro(3)-lado/2;
            centro(1)-lado/2, centro(2)+lado/2, centro(3)-lado/2;
            centro(1)-lado/2, centro(2)-lado/2, centro(3)+lado/2;
            centro(1)+lado/2, centro(2)-lado/2, centro(3)+lado/2;
            centro(1)+lado/2, centro(2)+lado/2, centro(3)+lado/2;
            centro(1)-lado/2, centro(2)+lado/2, centro(3)+lado/2];

% Definir la trayectoria con los 16 puntos (8 vértices y conexiones entre ellos)
trayectoria_indices = [1, 2, 3, 4, 1, 5, 6, 2, 6, 7, 3, 7, 8, 4, 8, 5, 1];
n_puntos = length(trayectoria_indices);

% Crear la trayectoria conectando los vértices
positions = zeros(n_puntos, 3);
for i = 1:n_puntos
    positions(i, :) = vertices(trayectoria_indices(i), :);
end

% Asignar posiciones calculadas
cons1 = positions(:, 1);
cons2 = positions(:, 2);
cons3 = positions(:, 3);

% Definir rotaciones fijas
rotations = repmat([90, 45, 0], n_puntos, 1);
cons4 = rotations(:, 1);
cons5 = rotations(:, 2);
cons6 = rotations(:, 3);

% Guardar las posiciones y rotaciones en un archivo de texto
archivo_salida_txt = 'trayectoria_cubo.txt';
fileID = fopen(archivo_salida_txt, 'w');

% Escribir la primera línea con la palabra 'cartesiano'
fprintf(fileID, 'cartesiano\n');

for i = 1:n_puntos
    fprintf(fileID, '%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%d,%.3f\n', ...
        cons1(i), cons2(i), cons3(i), cons4(i), cons5(i), cons6(i), 15, 0);
end
fclose(fileID);
disp(['Archivo guardado: ', archivo_salida_txt]);

% Graficar la trayectoria en 3D
figure;
plot3(cons1, cons2, cons3, 'o-');
xlabel('X (mm)'); 
ylabel('Y (mm)'); 
zlabel('Z (mm)');
title('Trayectoria en Cubo');
grid on; 
axis equal;

disp('Trayectoria en cubo generada y guardada correctamente.');
