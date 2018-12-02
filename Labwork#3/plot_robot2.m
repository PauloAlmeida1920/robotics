
%% PLOT do Robot com velocidades

function plot_robot2(robot, k, V, qVelocidades, q_controlo, pos)

    % Plot segundo o Controlador
    figure('units','normalized','outerposition',[0 0 1 1]);
    % Prespectiva de lado do Robot
    subplot(2,3,5);
    robot.plot(q_controlo(1,:), 'workspace', [-10 70 -20 40 -1 10], 'reach', ...
                                                    1, 'scale', 8, 'zoom', 0.25); 
    % Prespectiva de topo do Robot
    subplot(2,3,6);
    robot.plot(q_controlo(1,:), 'workspace', [-10 60 -10 60 -10 60], 'reach', ...
                                    1, 'scale', 8, 'zoom', 0.25, 'view', 'top'); % 'trail', 'b.');

    % Animação do Robot
    X = linspace(1, k-1, k);
      
    for i=1:k-1
        tic % Start a stopwatch timer
        
        robot.animate(q_controlo(i,:));

        % Plot das Velocidades Cartesianas
        subplot(2,3,1);
        v = plot( X(1:i), V(1:i,1)', '.',... % '-'
                  X(1:i), V(1:i,2)', 'o',... % 'o'
                  X(1:i), V(1:i,3)', '.');
        title('Velocidades Impostas');
        xlabel('k')
        ylabel('Velocidade Impostas (cm/s e rad/s)')
        xlim([0 k-1])
        ylim([-5 5])
        grid on

        % Plot das Velocidades das Juntas
        subplot(2,3,2);
        wth = plot( X(1:i), qVelocidades(1,1:i), '.',...
                    X(1:i), qVelocidades(2,1:i), '.',...
                    X(1:i), qVelocidades(3,1:i), '.');
        title('Velocidades das Juntas');
        xlabel('k')
        ylabel('Velocidade das Juntas (deg/s e cm/s)')
        xlim([0 k-1])
        grid on
        
        % Plot das Juntas
        subplot(2,3,3);
        th = plot( X(1:i), rad2deg(q_controlo(1:i,1)'), '.',...
                   X(1:i), q_controlo(1:i,2'), '.',...
                   X(1:i), rad2deg(q_controlo(1:i,3))', '.');
        title('Juntas');
        xlabel('k')
        ylabel('Valores Juntas (deg e cm)')
        xlim([0 k-1])
        %ylim([-1000 3500])
        grid on
        
        % Plot das Posições X Y Z        
        subplot(2,3,4);
        pos_ = plot( X(1:i), pos(1:i,1)', '.',...
                     X(1:i), pos(1:i,2)', '.',...
                     X(1:i), pos(1:i,3)', '.');
        title('Posições XYZ do gripper');
        xlabel('k')
        ylabel('Posição (cm)')
        xlim([0 k-1])
        ylim([-35 50])
        grid on

        %drawnow; %pause(0.05);  % this innocent line prevents the Matlab hang
        toc % Read the stopwatch timer

        % Temos que melhorar a perfomance destes plots!
    end
    
    legend(v, 'Vx', 'Vy', 'Wz');
    legend(wth, 'W theta1', 'V d2', 'W theta3');
    legend(th, 'Theta1', ' d2', 'Theta3');
    legend(pos_, 'X', 'Y', 'Z');

end