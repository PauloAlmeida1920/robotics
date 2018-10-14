close all
clear all
clc

disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robótica - 27/09/2018 ~ 14/10/2018] LABWORK#1 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


%% Sistema de coordenadas INICIAL do objecto no mundo localizado em:

Tw_0 = [ 0 -1  0  5;
         0  0 -1 -2;
         1  0  0  3;
         0  0  0  1 ];
         
         
%% Sequência de Movimentos Individual

% 1) Rotação de 30º sobre o eixo OX do sistema de coordenadas World:

T0_1 = trans(0, 0, 30, [0 0 0]') * Tw_0; % Pre-Multiplicação 


% 2) Deslocação de 3 unidades sobre o eixo OZ do atual sistema de coordenadas object:

T0_2 = T0_1 * trans(0, 0, 0, [0 0 3]'); % Pós-Multiplicação 


% 3) Rotação de -45º sobre o eixo [1 -1 1] do sistema de coordenadas object inicial: 

% Eixo (assumindo que o eixo está no referencial 0: posição inicial do objecto)
phi = deg2rad(-45);
r = [1 -1 1];

R = vectorRot(r, phi);


T2_3 = [ R [0 0 0 ]'; [0 0 0 1] ] * T0_2;


% 4) Rotação de 90º do sistema de coordenadas World sobre o seu próprio eixo OZ: (semelhante ao 1) )

T3_4 = trans(90, 0, 0, [0 0 0]') * T2_3; % Pre-Multiplicação 
         


%% Animação do Objecto
FPS = 10;

figure('units','normalized','outerposition',[0 0 1 1]);
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











