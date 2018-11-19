
%% Função da Cinemática Inversa do Robô
%  Consultar Ex1_Inversa.pdf, para melhor compreensão da implementação 

function [ q ] = inverse_kinematics_ex1(T0_G)

    % Cumprimento dos elos
    L1 = 40; L2 = 40; L3 = 10;
    
    % Vector t
    tx = T0_G(1,4);
    ty = T0_G(2,4);
    % vector a
    ax = T0_G(1,3); ay = T0_G(2,3);
    
  
    theta2 = -acos( (tx^2 + ty^2 - L1^2 - L2^2) / (2*L1*L2) );
    
    theta1 = atan2( ty, tx ) - atan2( L2*sin(theta2), L1 + L2*cos(theta2) );
    
    theta3 = atan2( ay, ax ) - theta1 - theta2; 
    
    
    q = [theta1 theta2 theta3];
    
end