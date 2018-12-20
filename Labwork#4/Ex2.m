clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%                   PLANEAMENTO DE TRAJECTÓRIAS                    %%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 28/11/2017 ~ 17/12/2017] LABWORK#4          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('*************************** Exercício 2 ******************************')

%% Manipulador	 do	 tipo	 2

syms theta1 theta2

% Comprimentos dos elos:
L1 = 1;
L2 = 1;

% Junta Rotacional ou Prismatica:
R = 1; P = 0;
%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1     L1      0      pi/2       0           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta2      0     L2        0        0           R ]; % Junta Rotacional
%_________________________________________________________________________________

% A cinematica directa da base   até ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti) ; % 3 matrizes: a última é uma transformação de corpo rígido( o gripper não tem junta)


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

% Tempo:
tA = 0; % instante de tempo no ponto A (instante inicial da trajectória)
tB = 8; % tempo final em [segundos], instante de tempo no ponto B (fim da trajectória)
ti = 5; % instante de tempo em que o manipulador passa no ponto intermédio, pi

% transformações para os pontos que definem a trajectória do mamnipulador

A_T_0 = [ 0   0.9659    0.2588      0         ;
          0   -0.2588   0.9659    sqrt(2)/2   ;
          1     0         0      (2-sqrt(2))/2;
          0     0         0         1        ];
      
B_T_0 = [ 0   0.9659    0.2588   sqrt(2)/2 ;
          0   -0.2588   0.9659   sqrt(2)/2 ;
          1     0         0         1      ;
          0     0         0         1      ];
      
C_T_0 = [ 0   0.8660     -0.5    sqrt(2)/2    ;
          0    0.5      0.8660      0         ;
          1     0         0      (2-sqrt(2))/2;
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
  
% Vector com os instantes de passagem nos pontos do percurso A->B->C
t = [ 0 4 10 ]; 

% Vector c/ valor das juntas nos pontos de passagens
q = [ qA; qB; qC ]; % Se fizermo para mais instantes é reptir estes pontos

% Cálcula as velocidades para cada junta em cada instante 
v_q = calcula_velocidade(q, t);

% Perío de amostragem 
h = 0.2;

% Cálcula trajectória para as respectivas juntas
q_aux = [ theta1 theta2 ];

[ pos, q_traj ] = calcula_trajectoria(oTg, t, q_aux, q, v_q, h);


%% MENU ("main")

% Variaveis MENU
select = 0;
STOP = 5;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'a) Pontos A B C',...
                                                   'b) A->B->C',...
                                                   'c) A->B->C->A->B->C',...
                                                   'd) Parabola',...
                                                   'Quit');
    
    %% a) Desenhe  as  configurações  do  manipulador  nos  três  pontos
    %     indique os respectivos valores das juntas para cada configuração
    if select == 1
        disp('______________________________________________________________________')
        disp('a) Configurações  do  manipulador  nos  três  pontos.')
        disp('______________________________________________________________________')
        
        disp(' ')
        disp(['Valores de qA = [ ' num2str(rad2deg(q(1,1))) 'º ' ...
            num2str(rad2deg(q(1,2))) 'º ] extraído através da cinemática inversa.'])
        disp(' ')
        disp(['Valores de qB = [ ' num2str(rad2deg(q(2,1))) 'º ' ...
            num2str(rad2deg(q(2,2))) 'º ] extraído através da cinemática inversa.'])
        disp(' ')
        disp(['Valores de qC = [ ' num2str(rad2deg(q(3,1))) 'º ' ...
            num2str(rad2deg(q(3,2))) 'º ] extraído através da cinemática inversa.'])
        disp(' ')
        
        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de lado do Robot
        subplot(1,2,1);
        
        plot3(A_T_0(1,4), A_T_0(2,4), A_T_0(3,4), 'ro');
        hold on
        plot3(B_T_0(1,4), B_T_0(2,4), B_T_0(3,4), 'r+');
        hold on
        plot3(C_T_0(1,4), C_T_0(2,4), C_T_0(3,4), 'rx');
        hold on
        xlim([-0.5 1])
        ylim([-0.5 1])
        zlim([-0.5 1.5])
        
        grid on
        
        robot.plot(q(1,:), 'workspace', [-0.5 1 -0.5 1 -0.5 1.5], ...
                   'scale', 0.5, 'zoom', 1); % 'view', 'top', 'trail', 'b.');
        
        % Prespectiva de topo do Robot -------------------------------------
        subplot(1,2,2);
        
        plot3(A_T_0(1,4), A_T_0(2,4), A_T_0(3,4), 'ro');
        hold on
        plot3(B_T_0(1,4), B_T_0(2,4), B_T_0(3,4), 'r+');
        hold on
        plot3(C_T_0(1,4), C_T_0(2,4), C_T_0(3,4), 'rx');
        hold on
        xlim([-0.5 1])
        ylim([-0.5 1])
        zlim([-0.5 1.5])
        
        grid on
        
        robot.plot(q(1,:), 'workspace', [-0.5 1 -0.5 1 -0.5 1.5],...
            'scale', 0.5,...
            'zoom', 1,...
            'view',...
            'top'); % 'trail', 'b.');
        pause(1)
        
        for i=2:1:3
            robot.animate(q(i,:));
            pause(1)
        end
        
        disp('#######################################################################')
    end
    %% b) Trajectória A-C em 10 segundos, A-B em 4 segundos
    if select == 2
        disp('______________________________________________________________________')
        disp('Trajectoria efectuada A->B->C pelo robô.')
        disp('______________________________________________________________________')
        
        % Vector com os instantes de passagem nos pontos do percurso A->B->C
        t = [ 0 4 10 ]; 

        % Vector c/ valor das juntas nos pontos de passagens
        q = [ qA; qB; qC ]; % Se fizermo para mais instantes é reptir estes pontos

        % Cálcula as velocidades para cada junta em cada instante
        v_q = calcula_velocidade(q, t);
        
        % Cálcula trajectória para as respectivas juntas
        [ pos, q_traj ] = calcula_trajectoria(oTg, t, q_aux, q, v_q, h);
        
        plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0);
        
        disp('#######################################################################')
    end % fim da alinea a)
    
    %% c) Movimento do manipulador segundo a trajectória A->B->C->A->B->C->...
    if select == 3
        disp('______________________________________________________________________')
        disp('Trajectoria A->B->C->A->B->C efectuada pelo robô.')
        disp('______________________________________________________________________')
        
        % Vector com os instantes de passagem nos pontos do percurso
        tAC = 2;
        t = [ 0 4 10 12 16 22 24 28 34 ];
        
        % Vector c/ valor das juntas nos pontos de passagens ; qB; qC
        q = [ qA; qB; qC; qA; qB; qC; qA; qB; qC ]; % Se fizermos para mais instantes é reptir estes pontos
        
        % Cálcula as velocidades para cada junta em cada instante
        v_q = calcula_velocidade(q, t);
        
        % Cálcula trajectória para as respectivas juntas
        [ pos, q_traj ] = calcula_trajectoria(oTg, t, q_aux, q, v_q, h);
        
        plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0);
        
        disp('#######################################################################')
    end
    
    %% d) Movimento do manipulador adoptando funções parabólicas
    if select == 4
        
       a_max = deg2rad(300);
        
       [ pos, q_traj ] = calcula_trajectoria2(oTg, a_max, q, t, h);
       
       plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0);
        
       disp('#######################################################################')
    end
    %
end


                   
                   
                   
                   
                   

