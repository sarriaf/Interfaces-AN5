
% Importar el modelo del robot desde el archivo URDF
robot = importrobot('fr5v6.urdf');
assignin('base', 'robot', robot);  % Guardar el robot en el workspace base

% Definir la configuración inicial del robot (opcional)
initialguess = robot.homeConfiguration;
initialguess(1).JointPosition = deg2rad(0);      % J1 = 0 grados
initialguess(2).JointPosition = deg2rad(-90);    % J2 = -90 grados (-pi/2)
initialguess(3).JointPosition = deg2rad(90);     % J3 = 90 grados (pi/2)
initialguess(4).JointPosition = deg2rad(-90);    % J4 = -90 grados (-pi/2)
initialguess(5).JointPosition = deg2rad(-90);    % J5 = -90 grados (-pi/2)
initialguess(6).JointPosition = deg2rad(90);     % J6 = 90 grados (pi/2)

% Visualización inicial del robot
show(robot, initialguess, 'Visuals', 'on');
