function qsalida = fr5_ik(posx_mm, posy_mm, posz_mm, orix_deg, oriy_deg, oriz_deg)
    % Variables persistent para mejorar rendimiento
    persistent robot ik initialguess;

    if isempty(robot)
        % Inicializar robot y configuración del solver
        [robot, ik] = initialize_robot();
    end

    % Convertir unidades de entrada
    posx = posx_mm / 1000; % mm a metros
    posy = posy_mm / 1000;
    posz = posz_mm / 1000;

    orix = deg2rad(orix_deg); % Grados a radianes
    oriy = deg2rad(oriy_deg);
    oriz = deg2rad(oriz_deg);

    % Validar si la posición es alcanzable antes de resolver IK
    if ~isPoseReachable(robot, posx, posy, posz)
        warning('La posición deseada está fuera del rango de trabajo del robot.');
        qsalida = NaN(1,6);
        return;
    end

    % Definir pesos para el solver de cinemática inversa
    weights = [1 1 1 1 1 1]; % Equilibrio entre posición y orientación

    % Definir la pose objetivo como una transformación homogénea (matriz 4x4)
    tform = trvec2tform([posx, posy, posz]) * eul2tform([oriz, oriy, orix]);
    %tform = trvec2tform([posx, posy, posz]) * eul2tform([orix, oriy, oriz]);


    % Configuración inicial FIJA: [0, -90, 90, -90, -90, 90] grados
    initialguess = robot.homeConfiguration;
    initialguess(1).JointPosition = 0;            % J1 = 0 radianes
    initialguess(2).JointPosition = -pi/2;        % J2 = -pi/2 radianes
    initialguess(3).JointPosition = pi/2;         % J3 = π/2 radianes
    initialguess(4).JointPosition = -pi/2;        % J4 = -π/2 radianes
    initialguess(5).JointPosition = -pi/2;        % J5 = -π/2 radianes
    initialguess(6).JointPosition = pi/2;         % J6 = π/2 radianes

    % Resolver la cinemática inversa
    [configSol, solInfo] = ik('tool_Link', tform, weights, initialguess);

    % Manejo de error en la solución
    if isempty(configSol) || solInfo.Status ~= "success"
        warning('No se encontró solución de cinemática inversa.');
        qsalida = NaN(1,6);
        return;
    end

    % Convertir resultados a grados
    qsalida = rad2deg(arrayfun(@(x) x.JointPosition, configSol));
end

function [robot, ik] = initialize_robot()
    % Cargar el modelo del robot desde un archivo URDF
    robot = importrobot(fullfile(pwd, 'fr5v6.urdf')); % Verifica la ruta correcta

    % Configurar el solver de cinemática inversa
    ik = inverseKinematics('RigidBodyTree', robot);

    % Configurar parámetros adicionales (evita reinicios aleatorios)
    ik.SolverParameters.AllowRandomRestart = false;
end

function isReachable = isPoseReachable(robot, px, py, pz)
    % Comprobar si la posición (px, py, pz) está dentro del rango del robot
    link_lengths = [0.425, 0.395, 0.109, 0.100]; % Longitudes de los eslabones del FR5
    max_reach = sum(link_lengths);
    min_reach = link_lengths(3); % Considera la distancia mínima por la base

    % Distancia al origen (asumiendo base en [0,0,0])
    distance = norm([px, py, pz]);

    % Validar si está dentro del rango de trabajo
    isReachable = (distance >= min_reach) && (distance <= max_reach);
end
% function qsalida = fr5_ik(posx_mm, posy_mm, posz_mm, orix_deg, oriy_deg, oriz_deg)
%     % Variables persistent para mejorar el rendimiento
%     persistent robot ik initialguess;
% 
%     if isempty(robot)
%         % Inicializar el robot y la configuración del solver
%         [robot, ik] = initialize_robot();
%     end
% 
%     % Convertir unidades de entrada: mm a metros y grados a radianes
%     posx = posx_mm / 1000; 
%     posy = posy_mm / 1000;
%     posz = posz_mm / 1000;
% 
%     orix = deg2rad(orix_deg); 
%     oriy = deg2rad(oriy_deg);
%     oriz = deg2rad(oriz_deg);
% 
%     % Validar si la posición está dentro de los límites de espacio de trabajo
%     if ~isPoseReachable(robot, posx, posy, posz)
%         warning('La posición deseada está fuera del rango de trabajo o de los límites predefinidos.');
%         qsalida = NaN(1,6);
%         return;
%     end
% 
%     % Definir pesos para el solver (equilibrio entre posición y orientación)
%     weights = [1 1 1 1 1 1];
% 
%     % Crear la pose objetivo (matriz de transformación homogénea)
%     % Nota: Se utiliza el orden [oriz, oriy, orix] para la conversión a Tform
%     tform = trvec2tform([posx, posy, posz]) * eul2tform([oriz, oriy, orix]);
% 
%     % Si es la primera llamada, inicializamos 'initialguess' con la configuración home
%     if isempty(initialguess)
%         initialguess = robot.homeConfiguration;
%         initialguess(1).JointPosition = 0;            % J1 = 0 radianes
%         initialguess(2).JointPosition = -pi/2;          % J2 = -pi/2 radianes
%         initialguess(3).JointPosition =  pi/2;          % J3 =  π/2 radianes
%         initialguess(4).JointPosition = -pi/2;          % J4 = -pi/2 radianes
%         initialguess(5).JointPosition = -pi/2;          % J5 = -pi/2 radianes
%         initialguess(6).JointPosition =  pi/2;          % J6 =  π/2 radianes
%     end
% 
%     % Resolver la cinemática inversa usando la solución previa como initial guess
%     [configSol, solInfo] = ik('tool_Link', tform, weights, initialguess);
% 
%     % Manejo de error si no se encontró solución
%     if isempty(configSol) || solInfo.Status ~= "success"
%         warning('No se encontró solución de cinemática inversa.');
%         qsalida = NaN(1,6);
%         return;
%     end
% 
%     % Convertir la solución a grados
%     qsalida = rad2deg(arrayfun(@(x) x.JointPosition, configSol));
% 
%     % Actualizar el initial guess con la solución actual para futuras llamadas
%     initialguess = configSol;
% end
% 
% function [robot, ik] = initialize_robot()
%     % Cargar el modelo del robot desde el archivo URDF
%     robot = importrobot(fullfile(pwd, 'fr5v6.urdf')); % Verifica la ruta correcta
%     % Configurar el solver de cinemática inversa
%     ik = inverseKinematics('RigidBodyTree', robot);
%     % Evitar reinicios aleatorios en el solver
%     ik.SolverParameters.AllowRandomRestart = false;
% end
% 
% function isReachable = isPoseReachable(robot, px, py, pz)
%     % Límites de posición (convertidos de mm a metros)
%     x_lim = [-830, -320] / 1000;  % en metros
%     y_lim = [-500, 500]  / 1000;
%     z_lim = [0, 720]     / 1000;
% 
%     % Comprobar que la posición esté dentro de los límites definidos
%     if (px < x_lim(1)) || (px > x_lim(2)) || ...
%        (py < y_lim(1)) || (py > y_lim(2)) || ...
%        (pz < z_lim(1)) || (pz > z_lim(2))
%         isReachable = false;
%         return;
%     end
% 
%     % Opcional: se puede incluir una verificación basada en la distancia (alcance)
%     % Longitudes de los eslabones (en metros) para el FR5 (ejemplo)
%     link_lengths = [0.425, 0.395, 0.109, 0.100];
%     max_reach = sum(link_lengths);
%     min_reach = link_lengths(3); % Distancia mínima (por ejemplo, considerando la geometría)
% 
%     distance = norm([px, py, pz]);
% 
%     % La posición se considera alcanzable si está dentro del rango [min_reach, max_reach]
%     isReachable = (distance >= min_reach) && (distance <= max_reach);
% end
