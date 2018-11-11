
%% Cinemática Inversa do Robô 2

function q = inverse_kinematics_robot1_follow_robot2(theta, d)

    % Offset/comprimentos dos elos
    l = 10;
    L1 = sqrt(2)*l;
    L2 = sqrt(2)*l;
    L3 = sqrt(7)*l;
    
    % Cinemática Inversa do Braço:


    tx = l*cos(theta) + (d)*sin(theta);
    ty = l*sin(theta) - (d)*cos(theta);
    tz = 0;
    
    theta2 = -acos( (tx^2 + ty^2 - L1^2 -L2^2)/(2*L1*L2) );
    theta1 = atan2( ty, tx ) - atan2( L2*sin(theta2), L1 + L2*cos(theta2) );
    theta3 = -pi/2 + theta - theta1 - theta2;
    
    
    % Valores das Juntas para o robô planar 1
    q = [theta1 theta2 theta3]; 

end










