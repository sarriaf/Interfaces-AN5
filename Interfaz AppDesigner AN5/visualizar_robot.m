function visualizar_robot(q1, q2, q3, q4, q5, q6)
    persistent robot trajectory_points fig ax;

    if isempty(robot)
        % Cargar el modelo del robot solo la primera vez
        robot = importrobot('fr5v6.urdf');  % Cargar el URDF
        
        % Crear la figura y axes para visualizar el robot
        fig = figure;
        ax = axes(fig);
        
        % Inicializar la matriz de puntos de la trayectoria
        trajectory_points = [];
    end

    % Configurar las articulaciones del robot con los valores de entrada
    config = robot.homeConfiguration;
    config(1).JointPosition = deg2rad(q1);
    config(2).JointPosition = deg2rad(q2);
    config(3).JointPosition = deg2rad(q3);
    config(4).JointPosition = deg2rad(q4);
    config(5).JointPosition = deg2rad(q5);
    config(6).JointPosition = deg2rad(q6);

    % Mostrar el robot en el axes creado
    show(robot, config, 'Parent', ax, 'PreservePlot', false);
    axis(ax, 'vis3d');          % Mantener las proporciones 3D
    view(ax, [18 20]);          % Establecer la vista predeterminada
    rotate3d(ax, 'on');         % Permitir la rotación interactiva

    % Obtener la posición del efector final
    tform = getTransform(robot, config, 'tool_Link');
    efector_pos = tform(1:3, 4);  % Extraer la posición (x, y, z)

    % Almacenar la posición del efector final para dibujar la trayectoria
    trajectory_points = [trajectory_points; efector_pos'];

    % Dibujar la trayectoria del efector final (en azul)
    hold(ax, 'on');
    plot3(ax, trajectory_points(:,1), trajectory_points(:,2), trajectory_points(:,3), 'b.-', 'MarkerSize', 10);
    hold(ax, 'off');

    % Actualizar la visualización
    drawnow;
end
