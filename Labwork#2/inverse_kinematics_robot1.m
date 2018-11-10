
%% Cinemática Inversa do Robô 1

function q = inverse_kinematics_robot1(T0_G)

    % Offset/comprimentos dos elos (fixos)
    L1 = 1; L2 = 1; L5 = 0.25;
   
    % vector n
    ny = T0_G(2,1);
    % vector s
    sy = T0_G(2,2);
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3); 
    % vector t
    ty = T0_G(2,4); tz = T0_G(3,4);
    
    
    % Cinemática Inversa das juntas do Braço: d1 e d2
    
    d1 = tz - L5*az - L1;
    d2 = ty - L5*ay - L2;

    
    % Cinemática Inversa das juntas do Punho Esférico: theta3 theta4 theta 5
    
    theta3 = atan2( -ax, -az);
    theta4 = atan2( -cos(theta3)*az - sin(theta3)*ax, ay);
    theta5 = atan2( sy, -ny);
    
    
    % Valores das Juntas para o robô planar 1
    q = [d1 d2 theta3 theta4 theta5]; 

end

