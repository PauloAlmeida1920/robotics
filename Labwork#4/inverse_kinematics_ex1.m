%% Função da Cinemática Inversa do Robot
%  Consultar Ex3_Inversa.pdf, para melhor compreensão da implementação

function [ q ] = inverse_kinematics_ex1(oTg)

% Vector t
tx = oTg(1,4);
ty = oTg(2,4);

% vector a
ax = oTg(1,3);
ay = oTg(2,3);

% expressões para a cinemática inversa:

theta1 = atan2(ty - 10 * ay, tx - 10 * ax);

d2 = cos(theta1) * ( tx - 10 * ax ) + sin(theta1) * ( ty - 10 * ay );

theta3 = atan2(ay, ax) - theta1;

% vector com parâmetros das juntas:

q = [ theta1 d2 theta3 ];

end