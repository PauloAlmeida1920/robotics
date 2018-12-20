
%% Função apra realizar o plot de um Robô com duas vistas 

function plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0)

% Plot segundo a Trajectória acima defenida
figure('units','normalized','outerposition',[0 0 1 1]);
% Prespectiva de lado do Robô
subplot(1,2,1);

plot3(pos(:,1), pos(:,2),  pos(:,3), 'b');
hold on
plot3(A_T_0(1,4), A_T_0(2,4), A_T_0(3,4), 'ro');
hold on
plot3(B_T_0(1,4), B_T_0(2,4), B_T_0(3,4), 'r+');
hold on
plot3(C_T_0(1,4), C_T_0(2,4), C_T_0(3,4), 'rx');
hold on
xlim([-0.5 1])
ylim([-0.5 1])
zlim([-0.5 1.5])

grid on

robot.plot(q_traj(1,:), 'workspace', [-0.5 1 -0.5 1 -0.5 1.5], ...
    'scale', 0.5, 'zoom', 1); % , 'trail', 'b.'); % 'view', 'top');

% Prespectiva de topo do Robô
subplot(1,2,2);

plot3(pos(:,1), pos(:,2),  pos(:,3), 'b');
hold on
plot3(A_T_0(1,4), A_T_0(2,4), A_T_0(3,4), 'ro');
hold on
plot3(B_T_0(1,4), B_T_0(2,4), B_T_0(3,4), 'r+');
hold on
plot3(C_T_0(1,4), C_T_0(2,4), C_T_0(3,4), 'rx');
hold on
xlim([-0.5 1])
ylim([-0.5 1])
zlim([-0.5 1.5])

grid on

robot.plot(q_traj(1,:), 'workspace', [-0.5 1 -0.5 1 -0.5 1.5], ...
    'scale', 0.5, 'zoom', 1, 'view', 'top'); % 'trail', 'b.');

for i=2:size(q_traj,1)
    
    robot.animate(q_traj(i,:));
    
end
        
        
end