function visualizar_robot_simulink(q1, q2, q3, q4, q5, q6)
    persistent robot trajectory_points fig ax;

    % Cargar el modelo del robot solo la primera vez
    if isempty(robot)
        robot = importrobot('fr5v6.urdf');  % Cargar el URDF
        trajectory_points = [];  % Inicializar la matriz de puntos de la trayectoria
    end

    % Configurar el tamaño de la pantalla y crear la figura
    if isempty(fig) || ~isvalid(fig)
        screenSize = get(0, 'ScreenSize');
        screenWidth = screenSize(3);
        screenHeight = screenSize(4);
        fig = figure('Name', 'Visualización en Simulink', ...
                     'Position', [screenWidth / 2, 1, screenWidth / 2, screenHeight]);
        ax = axes(fig);
        trajectory_points = []; % Reiniciar trayectoria al crear una nueva figura
    else
        % Limpiar solo los gráficos actuales sin eliminar la trayectoria acumulada
        cla(ax);
    end

    % Configurar las articulaciones del robot con los valores de entrada
    config = robot.homeConfiguration;
    config(1).JointPosition = deg2rad(q1);
    config(2).JointPosition = deg2rad(q2);
    config(3).JointPosition = deg2rad(q3);
    config(4).JointPosition = deg2rad(q4);
    config(5).JointPosition = deg2rad(q5);
    config(6).JointPosition = deg2rad(q6);

    % Mostrar el robot en el eje creado
    show(robot, config, 'Parent', ax, 'PreservePlot', false);
    axis(ax, 'vis3d');
    view(ax, [18 20]);
    ax.XLim = [-1 1];
    ax.YLim = [-1 1];
    ax.ZLim = [0 1.2];
    rotate3d(ax, 'on');

    % Añadir la cuadrícula en el plano horizontal
    hold(ax, 'on');
    x_lim = [-0.83, -0.32];  % Límites en metros
    y_lim = [-0.5, 0.5];     % Límites en metros
    z_plane = 0;             % Posición en Z donde va la cuadrícula

    % Crear líneas paralelas al eje Y
    x_lines = linspace(x_lim(1), x_lim(2), 10);
    for i = 1:length(x_lines)
        plot3(ax, [x_lines(i), x_lines(i)], y_lim, [z_plane, z_plane], 'r--', 'LineWidth', 1);
    end

    % Crear líneas paralelas al eje X
    y_lines = linspace(y_lim(1), y_lim(2), 10);
    for i = 1:length(y_lines)
        plot3(ax, x_lim, [y_lines(i), y_lines(i)], [z_plane, z_plane], 'r--', 'LineWidth', 1);
    end

    % Obtener la posición del efector final y almacenar la trayectoria
    tform = getTransform(robot, config, 'tool_Link');
    efector_pos = tform(1:3, 4);
    trajectory_points = [trajectory_points; efector_pos'];

    %=== Trazo de la trayectoria en azul (versión original) ===
    trayectoria_handle = plot3(ax, ...
        trajectory_points(:,1), ...
        trajectory_points(:,2), ...
        trajectory_points(:,3), ...
        'b-', ...
        'LineWidth', 1.5);

    %=== Dibujo de puntos: inicial (verde), final (magenta), intermedios (rojo) ===
    if ~isempty(trajectory_points)
        % Punto inicial en verde
        punto_inicial_handle = plot3(ax, ...
            trajectory_points(1,1), trajectory_points(1,2), trajectory_points(1,3), ...
            'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

        % Punto final en púrpura
        plot3(ax, ...
            trajectory_points(end,1), trajectory_points(end,2), trajectory_points(end,3), ...
            'mo', 'MarkerSize', 5, 'MarkerFaceColor', 'm');

        % Para la LEYENDA de los puntos intermedios en rojo,
        % necesitamos guardar un handle. Si NO hay puntos intermedios,
        % creamos un "plot oculto" para que igualmente aparezca en la leyenda.
        if size(trajectory_points, 1) > 2
            intermediate_points_handle = plot3(ax, ...
                trajectory_points(2:end-1,1), ...
                trajectory_points(2:end-1,2), ...
                trajectory_points(2:end-1,3), ...
                'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
        else
            % Crea un plot "falso" que no dibuja nada, solo para la leyenda
            intermediate_points_handle = plot3(ax, nan, nan, nan, ...
                'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'Visible', 'off');
        end
    end
    hold(ax, 'off');

    %=== Añadimos un tercer elemento a la leyenda para "puntos intermedios (rojo)" ===
    legend([punto_inicial_handle, trayectoria_handle, intermediate_points_handle], ...
        { 'Punto inicial (verde)', ...
          'Trayectoria recorrida (azul)', ...
          'Puntos intermedios (rojo)' }, ...
        'TextColor', 'black', 'Location', 'northeast', 'FontSize', 8);

    % Actualizar la visualización
    drawnow;
end
