clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%                   PLANEAMENTO DE TRAJECTÓRIAS                    %%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 28/11/2017 ~ 23/12/2018] LABWORK#4          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('*************************** Exercício 2 ******************************')

%% Manipulador

syms theta1 theta2

% Offset/comprimentos dos elos (fixos)
a = 1;
l = 1;

% Junta Rotacional ou Prismatica:
R = 1; P = 0;
%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1      0      0    -pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta2      a      0     pi/2     pi/2           R;  % Junta Rotacional
%_________________________________________________________________________________
                0      l      0        0        0           R ]; 
%_________________________________________________________________________________


% A cinematica directa da base até ao Gripper: 
[ T0_G, Ti ] = MGD_DH(PJ_DH);       


%% INICIALIZAÇÃO DO ROBOT: CRIAR LINKS

for i = 1 : size(PJ_DH,1)
    
    if PJ_DH(i,6) == R              % Juntas Rotacionais
        
        L(i) = Link('d',eval(PJ_DH(i,2)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [0 3/2*pi]);
    end
    
    if PJ_DH(i,6) == P              % Junta Prismática
        
        L(i) = Link('theta',eval(PJ_DH(i,1)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [25 50]);    
    end
end

robot = SerialLink(L, 'name', 'Robot Planar RRR');


%% VARIÁVEIS GLOBAIS 
% transformações para os pontos que definem a trajectória do mamnipulador

A_T_0 = [ 1     0         0   (1+sqrt(2))/2;
          0     1         0  (-1+sqrt(2))/2;
          1     0         1       sqrt(2)/2;
          0     0         0         1        ];
      
B_T_0 = [ 1     0         1   sqrt(2)/2;
          0     1         1           1;
          1     0         1      sqrt(2)/2;
          0     0         0         1        ];
      
C_T_0 = [ 1     0         0           -1;
          0     1         0    sqrt(2)/2;
          0     0         1   -sqrt(2)/2;
          0     0         0         1        ];
      
      
% CINEMÁTICA INVERSA

% OBTER A CONFIGURAÇÃO DO MANIPULADOR (VALOR DAS JUNTAS) CORRESPONDENTE PARA OS PONTOS DA 
% TRAJECTÓRIA DESEJADOS 

% Valores das juntas para o ponto A
[ qA ] = inverse_kinematics_ex2(A_T_0);

% Valores das juntas para o ponto intermédio (pi)
[ qB ] = inverse_kinematics_ex2(B_T_0);

% Valores das juntas para o ponto B
[ qC ] = inverse_kinematics_ex2(C_T_0);         
  

% Perío de amostragem 
h = 0.2;


%% d) Movimento do manipulador adoptando funções parabólicas
%  Trajectória em b) A-C em 10 segundos, A-B em 4 segundos

% Vector com os instantes de passagem nos pontos do percurso A->B->C
t = [ 0 4 10 ];

% Vector c/ valor das juntas nos pontos de passagens
q = [ qA; qB; qC ]; % Se fizermo para mais instantes é reptir estes pontos

a_max = deg2rad(60);


[ pos, q_traj ] = calcula_trajectoria2(T0_G, a_max, q, t, h);

plotRobot2(robot, pos, [q_traj zeros(size(q_traj,1),1)], A_T_0, B_T_0, C_T_0);




