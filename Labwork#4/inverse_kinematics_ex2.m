%% Função da Cinemática Inversa do Robot
%  Expressões fornecidas no enuciado

function [ q ] = inverse_kinematics_ex2(oTg)

d1 = 1;

% Vector t
tx = oTg(1,4);
ty = oTg(2,4);
tz = oTg(3,4);

% expressões para a cinemática inversa:

theta1 = atan2(ty, tx);

theta2 = atan2( tz - d1, tx*cos(theta1) + ty*sin(theta1));

% vector com parâmetros das juntas:

q = [ theta1 theta2 ];


end