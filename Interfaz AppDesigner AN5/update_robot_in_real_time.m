function update_robot_in_real_time()
    % Inicializa el robot en el workspace (suponiendo que ya está cargado)
    robot = evalin('base', 'robot');  % Recupera el modelo del robot desde el workspace

    % Iniciar la simulación y recuperar los resultados
    simOut = sim('CTC_carteciano_FR5');  % Cambia 'CTC_carteciano_FR5' por el nombre de tu modelo Simulink

    % Extraer los resultados de los ángulos articulares (resultados_de_q es la señal de Simulink con las configuraciones articulares)
    resultados_de_q = simOut.get('q');  % Cambia 'q' por el nombre real de la variable en tu modelo Simulink
    
    % Actualizar el robot en cada instante de la simulación
    for t = 1:length(resultados_de_q.Time)
        % Extraer los ángulos de articulación en el tiempo t
        q = resultados_de_q.Data(t, :);
        
        % Asignar los ángulos de las articulaciones al robot
        jointConfig = robot.homeConfiguration;
        for i = 1:6
            jointConfig(i).JointPosition = q(i);
        end
        
        % Actualizar la visualización del robot
        show(robot, jointConfig, 'Visuals', 'on', 'PreservePlot', false);
        drawnow;
        pause(0.05);  % Pausar brevemente para la visualización
    end
end
