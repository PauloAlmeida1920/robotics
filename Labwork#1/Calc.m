clear all;
clc

syms theta1 theta2 theta3

%% C�lculo auxiliares para a Inversa

% Prova de S1.S1 = S1^2
R = cos(theta1)*cos(theta1) % <=> cos(theta1)^2
R = simplify(R)
%%
% Prova de C1.C12 + S1.S12 = C2 != (C1^2).C2 + (S2^2).S2
R = cos(theta1)*cos(theta1 +theta2) + sin(theta1)*sin(theta1 +theta2)
R = simplify(R)
%%
% Prova de C1.C12 != (C1^2).C2
R = cos(theta1)*cos(theta1 +theta2)
R = simplify(R)
%%
% Prova de (C1^2) + (S1^2) = 1
R = (cos(theta1)^2) + (sin(theta1)^2)
R = simplify(R)
%%
% Prova de C12 != C1.C2 mas sim cos(1)*cos(2) - sin(1)*sin(2)
R = cos(theta1+theta2)
R = simplify(R) 
%%
% Prova de (C12) = C1.C2 - S1.S2 -> http://www2.clarku.edu/~djoyce/trig/identities.html
R = cos(theta1 + theta2)
R = simplify(R) 
% O Matlab n�o vai simplicar o que j� est� logo
% Vamo provar o contr�rio
R = cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2) 
R = simplify(R) 
%%
% Prova de (S12) = S1.C2 + C1.S2 
R = sin(theta1 + theta2)
R = simplify(R) 
% O Matlab n�o vai simplicar o que j� est� logo
% Vamo provar o contr�rio
R = sin(theta1)*cos(theta2) + cos(theta1)*sin(theta2) 
R = simplify(R) 
%%
% Prova de 
R = cos(theta1+theta2)
R = simplify(R) 
%% 
% Prova de (C12) = C1.C2 - S1.S2 
R = cos(theta1+theta2)*cos(theta2) + cos(theta1)*cos(theta2) + sin(theta1+theta2)*sin(theta2) + sin(theta1)*sin(theta2)
R = simplify(R) 
%%
% Prova de (C12) = C1.C2 - S1.S2 
R = cos(theta1)*cos(theta2) + sin(theta1)*sin(theta2) + cos(theta2)
R = simplify(R) 
%%
% Prova de (C12) = C1.C2 - S1.S2 
R = (cos(theta1)*cos(theta2))^2 + (sin(theta1)*sin(theta2))^2 + cos(theta1)^2 + sin(theta1)^2
R = simplify(R) 


%% Obten��o da Matriz corresponde ao theta1 c/ a oTg

% Cumprimentos dos elos
a2 = 40; d3 = 10;
% Junta Rotacional ou Prismatica
R = 1; P = 0;

%          thetai   di    ai   alfai  offseti  jointtypei
PJ_DH = [  theta1    0    a2       0        0           R;   % Junta Rotacional
           theta2    0    a2       0        0           R;   % Junta Rotacional
           theta3   d3     0       0        0           R ]; % Junta Rotacional + Gripper

% A cinematica directa ate o Gripper 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti = simplify(Ti);

% Cadeia Cinem�tica:
T01 = Ti(:,:,1);
T12 = Ti(:,:,2);
T2G = Ti(:,:,3);
clc
% Fun��o direct kinematics -adi��o de nova informa��o/posi��o/orienta��o
T01 = simplify(eye(4,4)*Ti(:,:,1))
T02 = simplify(T01*Ti(:,:,2))
T0G = simplify(T02*Ti(:,:,3))

% Ou simplesmente
T0G = simplify(T01*T12*T2G);
% d� errado
%T01 = simplify(inv(T12)*inv(T2G)*T0G);


% T01 = T0G - T12
T01_ = simplify(T0G - T01)


%% 
theta2 = atan2( 40*sin(pi), 40 + 40*cos(pi) )

%%
syms ty tx

R = cos(atan2(ty,tx))
R = simplify(R) 























