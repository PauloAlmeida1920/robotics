
%% Cinemática Inversa do Robô 1

function q = inverse_kinematics_robot1(T0_G)

    % Offset/comprimentos dos elos (fixos)
    L1 = 1; L2 = 1;
   
    % vector n
    ny = T0_G(2,1);
    % vector s
    sy = T0_G(2,2);
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3); 
    % vector t
    ty = T0_G(2,4); tz = T0_G(3,4);
    
    
    % Cinemática Inversa das juntas do Braço: theta1 theta2 theta3
    
    theta2 = acos(( tx^2 + ty^2 - L1^2 -L2^2)/( 2*L1*L2 ));
    
    theta1 = atan2( ty, tx ) - atan2( L3*sin(theta2), L1 + L2 *cos(theta2));
    
    theta3 = atan2(ay, ax) - theta1 - theta2;
    
    
    % Valores das Juntas para o robô planar 1
    q = [theta1 theta2 theta3]; 

end

