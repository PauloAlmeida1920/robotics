%% Função da Cinemática Inversa do Robot
%  Consultar Ex3_Inversa.pdf, para melhor compreensão da implementação

function [ q ] = inverse_kinematics_ex3(oTg, alfa)

% Vector t
tx = oTg(1,4);
ty = oTg(2,4);

% Com base no angulo em relação ao mundo - alfa:

theta1 = atan2( ty - 10 * cos(alfa), tx - 10 * sin(alfa));

theta3 = atan(cot(alfa)) - theta1;

d2 = cos(theta1) * (tx - 10 * sin(alfa)) + sin(theta1) * (ty - 10 * cos(alfa)); 

q = [ theta1 d2 theta3 ];

end