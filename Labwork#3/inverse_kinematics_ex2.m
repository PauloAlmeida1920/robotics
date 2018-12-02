  
%% Função da Cinemática Inversa do Robot
%  Consultar Ex2_Inversa.pdf, para melhor compreensão da implementaçao

function [ q ] = inverse_kinematics_ex2(T0_G, alfa)

% Com base no angulo em relação ao mundo - alfa:

% Vector n
nx = -cos(alfa); ny = sin(alfa);
% Vector a
ax = sin(alfa); ay = cos(alfa);

% Vector t
tx = T0_G(1,4); ty = T0_G(2,4);


% Cinemática inversa:

theta1 = atan2( ty - 10*ay, tx - 10*ax );

theta3 = atan2( -nx*cos(theta1) - ny*sin(theta1), ny*cos(theta1) - nx*sin(theta1) );

d2 = sqrt( (tx - 10*ax)^2 + (ty - 10*ay)^2 ); 



q = [ theta1 d2 theta3 ];

end