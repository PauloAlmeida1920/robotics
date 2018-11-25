
%% PLOT do Robô com velocidades

function plot_robot(robot, k, V, qVelocidades, q_controlo)

    % Plot segundo o Controlador
    figure('units','normalized','outerposition',[0 0 1 1]);
    % Prespectiva de lado do Robô
    subplot(2,3,[4 5 6]);
    %hold off
    robot.plot(q_controlo(1,:), 'workspace', [-60 60 -60 60 -1 10], 'reach', ... % [-40 60 -10 100 -10 70]
        1, 'scale', 8, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
    % Prespectiva de topo do Robô
%    subplot(2,3,4);
    %hold on
%     robot.plot(q_controlo(1,:), 'workspace', [-40 60 -10 60 -10 60], 'reach', ...
%         1, 'scale', 8, 'zoom', 0.25, 'view', 'top'); % 'trail', 'b.');

    % Animação do robô
    X = linspace(1, k-1, k);

    for i=1:k-1
        tic % Start a stopwatch timer
        robot.animate(q_controlo(i,:));
        %end
        %figure(2);%'units','normalized','outerposition',[0 0 1 1]);
        %for i=1:k-1
        % Plot das Velocidades Cartesianas
        subplot(2,3,1);
        v = plot( X(1:i), V(1:i,1)', '.',... % '-'
                  X(1:i), V(1:i,2)', 'o');%,...
                  %X(1:i), V(1:i,3)', '.');
        title('Velocidades Cartesianas');
        %legend('Vx', 'Vy', 'Vz');
        xlabel('k')
        ylabel('Velocidade Cartesiana (m/s)')
        xlim([0 k-1])
        %ylim([-15 -3])
        grid on

        % Plot das Velocidades das Juntas
        subplot(2,3,2);
        wth = plot( X(1:i), qVelocidades(1,1:i), '.',...
                    X(1:i), qVelocidades(2,1:i), '.',...
                    X(1:i), qVelocidades(3,1:i), '.');
        title('Velocidades das Juntas');
        %legend('W theta1', 'W theta2', ' W theta3');
        xlabel('k')
        ylabel('Velocidade de Juntas (rad/s)')
        xlim([0 k-1])
        grid on

        % Plot das Juntas
        subplot(2,3,3);
        th = plot( X(1:i), rad2deg(q_controlo(1:i,1)'), '.',...
                   X(1:i), rad2deg(q_controlo(1:i,2)'), '.',...
                   X(1:i), rad2deg(q_controlo(1:i,3)'), '.');
        title('Angulos das Juntas');
        %legend('Theta1', 'Theta2', ' Theta3');
        xlabel('k')
        ylabel('Angulos Juntas (graus)')
        xlim([0 k-1])
        grid on

        %drawnow; %pause(0.05);  % this innocent line prevents the Matlab hang
        toc % Read the stopwatch timer

        % Temos que melhorar a perfomance destes plots!
    end
    
    legend(v, 'Vx', 'Vy'); %, 'Vz');
    legend(wth, 'W theta1', 'W theta2', 'W theta3');
    legend(th, 'Theta1', 'Theta2', ' Theta3');

end