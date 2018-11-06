
%% Função da Cinemática Inversa do Robô
function [ q_pos, q_neg ] = inverse_kinematics_ex2(oTh)

    % Cumprimento do elo 3
    L3 = 2;
    
    % T02 a partir da T0H:
    % T02 = T0H - L3*R0G,oz, alfa = 0º
    tx_ = oTh(1,4) - L3*oTh(1,3);
    ty_ = oTh(2,4) - L3*oTh(2,3);
    
    num = tx_^2 + ty_^2 - 3^2 - 4^2;
    den = 2*3*4;
    
    
    % Caso em que theta2 positivo
    
    % acos pois     tx^2 + ty^2 = 3^2 + 4^2 + 2.3.4.C2 
    %           <=> C2 = tx^2 + ty^2 - 3^2 - 4^2)/2.3.4
    theta2 = acos((tx_^2 + ty_^2 - 3^2 - 4^2)/(2*3*4));
    % segundo atan2(ty,tx)
    theta1 = atan2(ty_,tx_) - atan2(3*sin(theta2), 4+3*cos(theta2));
    % theta3 = alfa - theta2 - theta1
    theta3 = atan2(oTh(2,3), oTh(1,3)) - theta1 - theta2;
    
    q_pos = [theta1 theta2 theta3];
    
    
    % Caso em que theta2 negativo
    
    theta2 = -acos((tx_^2 + ty_^2 - 3^2 - 4^2)/(2*3*4));
    % segundo atan2(ty,tx)
    theta1 = atan2(ty_,tx_) - atan2(3*sin(theta2), 4+3*cos(theta2));
    % theta3 = alfa - theta2 - theta1
    theta3 = atan2(oTh(2,3), oTh(1,3)) - theta1 - theta2;
    
    q_neg = [theta1 theta2 theta3];
    
end