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
wTobject = [ 0 -1  0  5;
             0  0 -1 -2;
             1  0  0  3;
             0  0  0  1 ];
         
%% Sequ�ncia de Movimentos Individual

% 1) Rota��o de 30� sobre o eixo OX do sistema de coordenadas World:

wTo_1 = trans(0, 0, 30, [0 0 0]') * wTobject;


% 2) Desloca��o de 3 unidades sobre o eixo OZ do atual sistema de coordenadas object:

wTo_2 = trans(0, 0, 0, [0 0 3]') * wTo_1;


% 3) Rota��o de -45� sobre o eixo [-1 -1 1] do sistema de coordenadas object inicial: 

% Eixo 
phi = deg2rad(-45);
r = [-1 -1 -1];

R = vectorRot(r, phi);

wTo_3 = [ R [0 0 0 ]'; [0 0 0 1] ] * wTobject;


% 4) Rota��o de 90� do sistema de coordenadas World sobre o seu pr�prio eixo OZ: (semelhante ao 1) )

wTo_4 = trans(90, 0, 0, [0 0 0]') * wTo_2;
         


%% Anima��o do Objecto

figure(3);
tranimate(wTobject, wTo_1, 'rgb', 'axis', [0 10 -10 10 0 10]);
hold on
tranimate(wTo_1, wTo_2, 'rgb', 'axis', [0 10 -10 0 10 10]);
hold on
tranimate(wTo_2, wTo_4, 'rgb', 'axis', [0 10 -10 0 10 10]);
hold on




%wTo_ = wTo_3 * trans(0, 0, 30, [0 0 3]');
%%
%wTo_ = wTo_3 * wTobject;

tranimate(wTobject, wTo_3, 'rgb', 'axis', [0 10 -10 0 -5 10]);












