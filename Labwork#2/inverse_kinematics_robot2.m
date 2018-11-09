
%% Cinemática Inversa do Robô 1

function q = inverse_kinematics_robot2(T0_G)

    % Offset/comprimentos dos elos (fixos)
    L1 = 1; L2 = 0.5; L5 = 0.25;
   
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3); 
    % vector t
    tx = T0_G(2,4); ty = T0_G(2,4); tz = T0_G(3,4);
    
    
    % Cinemática Inversa do Braço:
 
    d2 = sqrt( (tx - L5*ax)^2 + (ty - L5*ay)^2 - L1^2) - L2;
    
    theta1 = atan2(ty - L5*ay, tx - L5*ax) - atan2(L1, d2);
    
    
    % Cinemática Inversa do Punho Esférico:
    theta3 = atan2(-ax,-az);
    theta4 = atan2(-az*cos(theta3)-ax*sin(theta3), ay);
    theta5 = atan2(-ny, -sy);
    
    
    % Valores das Juntas para o robô planar 1
    q = [theta1 d2 theta3 theta4 theta5]; 

end










