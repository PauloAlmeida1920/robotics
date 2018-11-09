
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
    
    % Cinemática Inversa do Braço:
    
    d1 = tz - L5*az - L1;
    d2 = ty - L5*ay - L2;

    
    % Cinemática Inversa do Punho Esférico
    
    theta3 = atan2(-ax,-az);
    theta4 = atan2(-az*cos(theta3)-ax*sin(theta3), ay);
    theta5 = atan2(-ny, -sy);
    
    
    % Valores das Juntas para o robô planar 1
    q = [d1 d2 theta3 theta4 theta5]; 

end

% Sampaio
% 
%     T0_1 = eval(subs(Ti(:,:,1), d1)); 
%     T1_2 = eval(subs(Ti(:,:,2), d2)); 
% 
%     T0_2 = T0_1 * T1_2;
% 
%     T2_0 = inv(T0_2);
% 
%     T2_G = T2_0 * T0_G;
%     
%     % vector a
%     ax = T2_G(1,3); ay = T2_G(2,3); az = T2_G(3,3);
%     % vector s
%     sz = T2_G(3,2);
%     % vector n
%     nz = T2_G(3,1);
% 
%     theta3 = atan2( ay, ax );
%     theta4 = atan2( ax*cos(theta3) + ay*sin(theta3), az );
%     theta5 = atan2( sz, -nz );


% % vector n
% nx = T0_G(1,1); ny = T0_G(2,1); nz = T0_G(3,1);
% % vector s
% sx = T0_G(1,2); sy = T0_G(2,2); sz = T0_G(3,2);
% % vector a
% ax = T0_G(1,3); ay = T0_G(2,3); az = T0_G(3,3);
% % vector t
% tx = T0_G(1,4); ty = T0_G(2,4); tz = T0_G(3,4);










