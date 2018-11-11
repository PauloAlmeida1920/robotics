
%% Cinemática Inversa do Robô 2

function q = inverse_kinematics_robot3(T0_G)

    % Offset/comprimentos dos elos (fixos)
    L2 = 1; L4 = 1; L6 = 0.25;
   
    % vector n
    nx = T0_G(1,1); ny = T0_G(2,1);
    % vector s
    sx = T0_G(1,2); sy = T0_G(2,2);
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3); 
    % vector t
    tx = T0_G(1,4); ty = T0_G(2,4);
    
    
    tx_ = tx - L4*ax;
    ty_ = ty - L4*ay;
    
   
    % Cinemática Inversa do Braço:
    
    theta1 = atan2(tx_, -ty_);
    
    d2 = tx_*sin(theta1) - ty_*cos(theta1) - L2 -0.65; % 0.65 é um erro asssociado
                                                       % verificamos o mesmo para
                                                       % vários valores da junta d2 
                                                    
    theta3 = atan2( tx - (d2 + L2)*sin(theta1), -ty - (d2 + L2)*cos(theta1) ) - theta1 - 0.1283;
    
    
    % Cinemática Inversa do Punho Esférico:
    
    theta4 = atan2( az, ax*cos(theta1 + theta3) + ay*sin(theta1 + theta3) );
    
    theta5 = atan2( cos(theta4)*( ax*cos(theta1 + theta3) + ay*sin(theta1 + theta3) ) ...
                    + az*sin(theta4), ax*sin(theta1 + theta3) - ay*cos(theta1 + theta3) );
    
    theta6 = atan2( sx*sin(theta1 + theta3) - sy*cos(theta1 + theta3), ...
                    -(nx*sin(theta1 + theta3) - ny*cos(theta1 + theta3)) );

    
    % Valores das Juntas para o robô planar 1
    q = [theta1 d2 theta3 theta4 theta5 theta6]; 

end


    








