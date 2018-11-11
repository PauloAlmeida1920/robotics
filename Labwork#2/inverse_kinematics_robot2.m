
%% Cinemática Inversa do Robô 2

function q = inverse_kinematics_robot2(T0_G)

    % Offset/comprimentos dos elos (fixos)
    L1 = 1; L2 = 0.5; L5 = 0.25;
    
    % vector n
    nx = T0_G(1,1); ny = T0_G(2,1);
    % vector s
    sx = T0_G(1,2); sy = T0_G(2,2);
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3); 
    % vector t
    tx = T0_G(1,4); ty = T0_G(2,4);
    
    
    % Cinemática Inversa do Braço:
 
    d2 = sqrt( (tx - L5*ax)^2 + (ty - L5*ay)^2 - L1^2 ) - L2;
    
    theta1 = atan2(ty - L5*ay, tx - L5*ax) + atan2( L1, d2 + L2);
    
    
    % Cinemática Inversa do Punho Esférico:
    
    theta3 = atan2( -az, ax*sin(theta1) - ay*cos(theta1) );
    
    theta4 = atan2( cos(theta3)*(ax*sin(theta1) - ay*cos(theta1)) ...
                    - az*sin(theta3), ax*cos(theta1) + ay*sin(theta1) );
                
    theta5 = -atan2( -sx*cos(theta1) - sy*sin(theta1), -nx*cos(theta1) - ny*sin(theta1) );
    
    
    % Valores das Juntas para o robô planar 1
    q = [theta1 d2 theta3 theta4 theta5]; 

end










