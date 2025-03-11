% Importar el modelo del robot desde un archivo URDF
robot = importrobot('fr5v6.urdf');
showdetails(robot); % Ver detalles del robot

% Visualizar el robot en su configuración inicial
figure;
show(robot);
title('Configuración Inicial del Robot');

% Solicitar al usuario los valores de theta (articulaciones) en grados
theta1 = deg2rad(input('Ingrese Theta 1 en grados: '));
theta2 = deg2rad(input('Ingrese Theta 2 en grados: '));
theta3 = deg2rad(input('Ingrese Theta 3 en grados: '));
theta4 = deg2rad(input('Ingrese Theta 4 en grados: '));
theta5 = deg2rad(input('Ingrese Theta 5 en grados: '));
theta6 = deg2rad(input('Ingrese Theta 6 en grados: '));

% Definir la configuración del robot en base a los valores de theta proporcionados
config = robot.homeConfiguration;
config(1).JointPosition = theta1;
config(2).JointPosition = theta2;
config(3).JointPosition = theta3;
config(4).JointPosition = theta4;
config(5).JointPosition = theta5;
config(6).JointPosition = theta6;

% Calcular la cinemática directa utilizando getTransform
tform = getTransform(robot, config, 'tool_Link');
% Extraer la posición (x, y, z) en metros
position = tform2trvec(tform);

% Mostrar la matriz de rotación del "end-effector"
rotm = tform2rotm(tform);
ry = rad2deg(atan2(-rotm(3, 1), sqrt(rotm(1, 1)^2 + rotm(2, 1)^2)));
rx = rad2deg(atan2(rotm(3, 2), rotm(3, 3)));
rz = rad2deg(atan2(rotm(2, 1), rotm(1, 1)));
disp('Matriz de rotación del end-effector:');
disp(rotm);


% Extraer la orientación (roll, pitch, yaw) en radianes y convertir a grados
orientation_rad = tform2eul(tform);
orientation_deg = rad2deg(orientation_rad);

% % Ajustar yaw para que sea 0 cuando sea equivalente a -180
% if abs(orientation_deg(3) + 180) < 1e-2
%     orientation_deg(3) = 0;
% end
% 
% % Ajustar roll para que se muestre correctamente en el rango [-180, 180]
% if orientation_deg(1) < 0
%     orientation_deg(1) = orientation_deg(1) + 180;
% elseif orientation_deg(1) > 0
%     orientation_deg(1) = orientation_deg(1) - 180;
% end

% Convertir la posición a milímetros
position_mm = position * 1000;

% Mostrar resultados
fprintf('Posición del end-effector (x, y, z) en mm: [%.2f, %.2f, %.2f]\n', position_mm);
fprintf('Orientación del end-effector (roll, pitch, yaw) en grados: [%.2f, %.2f, %.2f]\n', orientation_deg);
fprintf('Orientación del end-effector (roll, pitch, yaw) en grados: [%.2f, %.2f, %.2f]\n', rx,ry,rz);

% Visualizar el robot en la configuración resultante
figure;
show(robot, config);
title('Configuración del Robot en Base a Thetas Ingresadas');

% Ploteo de la posición final del end-effector
figure;
hold on;
plot3(position_mm(1), position_mm(2), position_mm(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Posición Final del End-Effector');
grid on;
view(3);
hold off;
