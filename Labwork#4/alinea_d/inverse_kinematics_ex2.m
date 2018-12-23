%% Função da Cinemática Inversa do Robot
%  Expressões fornecidas no enuciado

function [ q ] = inverse_kinematics_ex2(oTg)

a = 1; l = 1;
d1 = 1;

% Vector t
tx = oTg(1,4);
ty = oTg(2,4);
tz = oTg(3,4);


% % expressões para a cinemática inversa:
% theta2 =  atan2( -tz, sqrt(tx^2 + ty^2 - 1));
%  
% theta1 = atan2(ty, tx) - atan2(1, cos(theta2)); 

theta1 = atan2(ty, tx);

theta2 = atan2( tz - d1, tx*cos(theta1) + ty*sin(theta1));


% vector com parâmetros das juntas:

q = [ theta1 theta2 ];


end