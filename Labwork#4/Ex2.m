clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%                   PLANEAMENTO DE TRAJECT�RIAS                    %%')
disp('%%                                                                  %%')
disp('%%          [Rob�tica - 28/11/2017 ~ 17/12/2017] LABWORK#4          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, n� 2011283029                   %%')
disp('%%                   Paulo Almeida, n� 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('*************************** Exerc�cio 2 ******************************')

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
PJ_DH = [  theta1     0      0     -pi/2       0           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta2     a      0      pi/2      pi/2         R;   % Junta Rotacional
%_________________________________________________________________________________
           0          L      0        0        0           R ]; 
%_________________________________________________________________________________

% A cinematica directa da base   at� ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti) ; % 3 matrizes: a �ltima � uma transforma��o de corpo r�gido( o gripper n�o tem junta)


%% INICIALIZA��O DO ROBOT: CRIAR LINKS

for i = 1 : size(PJ_DH,1)
    
    if PJ_DH(i,6) == R              % Juntas Rotacionais
        
        L(i) = Link('d',eval(PJ_DH(i,2)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [0 3/2*pi]);
    end
    
    if PJ_DH(i,6) == P              % Junta Prism�tica
        
        L(i) = Link('theta',eval(PJ_DH(i,1)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [25 50]);    
    end
end

robot = SerialLink(L, 'name', 'Robot Planar RRR');


%% VARI�VEIS GLOBAIS 

% Tempo:
tA = 0; % instante de tempo no ponto A (instante inicial da traject�ria)
tB = 8; % tempo final em [segundos], instante de tempo no ponto B (fim da traject�ria)
ti = 5; % instante de tempo em que o manipulador passa no ponto interm�dio, pi

% transforma��es para os pontos que definem a traject�ria do mamnipulador

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
    
      
% CINEM�TICA INVERSA

% OBTER A CONFIGURA��O DO MANIPULADOR (VALOR DAS JUNTAS) CORRESPONDENTE PARA OS PONTOS DA 
% TRAJECT�RIA DESEJADOS 

% Valores das juntas para o ponto A
[ qA ] = inverse_kinematics_ex2(A_T_0);

% Valores das juntas para o ponto interm�dio (pi)
[ qB ] = inverse_kinematics_ex2(B_T_0);

% Valores das juntas para o ponto B
[ qC ] = inverse_kinematics_ex2(C_T_0);         
  
% Vector com os instantes de passagem nos pontos do percurso A->B->C
t = [ 0 4 10 ]; 

% Vector c/ valor das juntas nos pontos de passagens
q = [ qA; qB; qC ]; % Se fizermo para mais instantes � reptir estes pontos

% C�lcula as velocidades para cada junta em cada instante 
v_q = calcula_velocidade(q, t);

% Per�o de amostragem 
h = 0.2;

% C�lcula traject�ria para as respectivas juntas
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
    
    %% a) Desenhe  as  configura��es  do  manipulador  nos  tr�s  pontos
    %     indique os respectivos valores das juntas para cada configura��o
    if select == 1
        disp('______________________________________________________________________')
        disp('a) Configura��es  do  manipulador  nos  tr�s  pontos.')
        disp('______________________________________________________________________')
        
        disp(' ')
        disp(['Valores de qA = [ ' num2str(rad2deg(q(1,1))) '� ' ...
            num2str(rad2deg(q(1,2))) '� ] extra�do atrav�s da cinem�tica inversa.'])
        disp(' ')
        disp(['Valores de qB = [ ' num2str(rad2deg(q(2,1))) '� ' ...
            num2str(rad2deg(q(2,2))) '� ] extra�do atrav�s da cinem�tica inversa.'])
        disp(' ')
        disp(['Valores de qC = [ ' num2str(rad2deg(q(3,1))) '� ' ...
            num2str(rad2deg(q(3,2))) '� ] extra�do atrav�s da cinem�tica inversa.'])
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
    %% b) Traject�ria A-C em 10 segundos, A-B em 4 segundos
    if select == 2
        disp('______________________________________________________________________')
        disp('Trajectoria efectuada A->B->C pelo rob�.')
        disp('______________________________________________________________________')
        
        % Vector com os instantes de passagem nos pontos do percurso A->B->C
        t = [ 0 4 10 ]; 

        % Vector c/ valor das juntas nos pontos de passagens
        q = [ qA; qB; qC ]; % Se fizermo para mais instantes � reptir estes pontos

        % C�lcula as velocidades para cada junta em cada instante
        v_q = calcula_velocidade(q, t);
        
        % C�lcula traject�ria para as respectivas juntas
        [ pos, q_traj ] = calcula_trajectoria(oTg, t, q_aux, q, v_q, h);
        
        plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0);
        
        disp('#######################################################################')
    end % fim da alinea a)
    
    %% c) Movimento do manipulador segundo a traject�ria A->B->C->A->B->C->...
    if select == 3
        disp('______________________________________________________________________')
        disp('Trajectoria A->B->C->A->B->C efectuada pelo rob�.')
        disp('______________________________________________________________________')
        
        % Vector com os instantes de passagem nos pontos do percurso
        tAC = 2;
        t = [ 0 4 10 12 16 22 24 28 34 ];
        
        % Vector c/ valor das juntas nos pontos de passagens ; qB; qC
        q = [ qA; qB; qC; qA; qB; qC; qA; qB; qC ]; % Se fizermos para mais instantes � reptir estes pontos
        
        % C�lcula as velocidades para cada junta em cada instante
        v_q = calcula_velocidade(q, t);
        
        % C�lcula traject�ria para as respectivas juntas
        [ pos, q_traj ] = calcula_trajectoria(oTg, t, q_aux, q, v_q, h);
        
        plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0);
        
        disp('#######################################################################')
    end
    
    %% d) Movimento do manipulador adoptando fun��es parab�licas
    if select == 4
        
       a_max = deg2rad(300);
        
       [ pos, q_traj ] = calcula_trajectoria2(oTg, a_max, q, t, h);
       
       plotRobot2(robot, pos, q_traj, A_T_0, B_T_0, C_T_0);
        
       disp('#######################################################################')
    end
    %
end


                   
                   
                   
                   
                   

