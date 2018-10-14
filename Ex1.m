close all
clear all
clc

disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Rob�tica - 12/09/2018 ~ 14/10/2018] LABWORK#1 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, n� 2011283029                    %%')
disp('%%                   Paulo Almeida, n� 2010128473                    %%')
disp('%%                                                                   %%')
disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


%% Sistema de coordenadas INICIAL do objecto no mundo localizado em:

Tw_0 = [ 0 -1  0  5;
         0  0 -1 -2;
         1  0  0  3;
         0  0  0  1 ];
         
         
%% Sequ�ncia de Movimentos Individual

% 1) Rota��o de 30� sobre o eixo OX do sistema de coordenadas World:

T0_1 = trans(0, 0, 30, [0 0 0]') * Tw_0; % Pre-Multiplica��o 


% 2) Desloca��o de 3 unidades sobre o eixo OZ do atual sistema de coordenadas object:

T0_2 = T0_1 * trans(0, 0, 0, [0 0 3]'); % P�s-Multiplica��o 


% 3) Rota��o de -45� sobre o eixo [1 -1 1] do sistema de coordenadas object inicial: 

% Eixo (assumindo que o eixo est� no referencial 0: posi��o inicial do objecto)
phi = deg2rad(-45);
r = [1 -1 1];

R = vectorRot(r, phi);


T2_3 = [ R [0 0 0 ]'; [0 0 0 1] ] * T0_2;


% 4) Rota��o de 90� do sistema de coordenadas World sobre o seu pr�prio eixo OZ: (semelhante ao 1) )

T3_4 = trans(90, 0, 0, [0 0 0]') * T2_3; % Pre-Multiplica��o 
         


%% Anima��o do Objecto
FPS = 10;

figure(3);
trplot(Tw_0,'rgb', 'axis', [-10 10 -10 10 -10 10]);
hold on
tranimate(Tw_0, T0_1, 'rgb', 'axis', [-10 10 -10 10 -10 10],'fps', FPS);
hold on
tranimate(T0_1, T0_2, 'rgb', 'axis', [-10 10 -10 10 -10 10],'fps', FPS);
hold on
% Eixo 
plot3([Tw_0(1,4) Tw_0(1,4)+1], [Tw_0(2,4) Tw_0(2,4)-1], [Tw_0(3,4) Tw_0(3,4)+1], 'm-');
hold on
tranimate(T0_2, T2_3, 'rgb', 'axis', [-10 10 -10 10 -10 10],'fps', FPS);
hold on
tranimate(T2_3, T3_4, 'rgb', 'axis', [-10 10 -10 10 -10 10],'fps', FPS);
hold on











