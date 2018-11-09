
%% Cinemática Inversa do Robô 2

function q = inverse_kinematics_robot3(T0_G)

    % Offset/comprimentos dos elos (fixos)
    L2 = 1; L4 = 1; L5 = 0.25;
   
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3); 
    % vector t
    tx = T0_G(2,4); ty = T0_G(2,4);
    
    
    % Cinemática Inversa do Braço:
    
    theta1 = atan2(tx - L4*ax, -(ty - L4*ay));
    
    d2 = (tx - L4*ax)*sin(theta1) - (ty - L4*ay)*cos(theta1) - L2;
    
    theta3 = atan2(tx - (d2 + L2)*sin(theta1), -ty - (d2 + L2)*cos(theta1)) - theta1;
    
    
    % Cinemática Inversa do Punho Esférico:
    
    theta4 = atan2(-ax, -az);
    theta5 = atan2(-az*cos(theta3) - ax*sin(theta3), ay);
    theta6 = atan2(-ny, -sy);
    
    
    % Valores das Juntas para o robô planar 1
    q = [theta1 d2 theta3 theta4 theta5 theta6]; 

end










