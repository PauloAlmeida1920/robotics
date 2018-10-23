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


%% Robot 3-DOF: RRR-RR

syms theta1 theta2 theta3 theta4 theta5

% Referencial Inicial 0
T0 = [ 1  0  0  0;
       0  1  0  0;
       0  0  1  0;
       0  0  0  1 ];

% Junta Rotacional ou Prismatica:
R = 1; P = 0;
%______________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei | range_min | range_max         
%______________________________________________________________________________________
PJ_DH = [  theta1      0      0     pi/2     pi/2           R       -pi/2       pi/2;   % Junta Rotacional
%______________________________________________________________________________________
           theta2      0      4        0        0           R       -pi/3       pi/4;   % Junta Rotacional
%______________________________________________________________________________________
           theta3      0      2        0        0           R       -pi/2       pi/2;   % Junta Rotacional
%______________________________________________________________________________________
           theta4      0      0    -pi/2    -pi/2           R       -pi/2       pi/2;   % Junta Rotacional
%______________________________________________________________________________________
           theta5      1      0        0        0           R       -pi         pi; ];  % Junta Rotacional
%______________________________________________________________________________________

% A cinematica directa da base   até ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti);


%% MENU ("main")

% Variaveis MENU
select = 0;
STOP = 4;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'Cinematica Directa',...
                                                   'Home',...
                                                   'Animação',...
                                                   'Quit');  
                                                
    %% Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('PJ_DH: Matriz dos parametros de Denavith-Hartenberg:')
        disp('______________________________________________________________________')
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) oTg: Cinematica Directa c/ variaveis simbolicas:')
        disp('______________________________________________________________________')
        disp(' ')
        disp(oTg)
        disp(' ')
        disp('______________________________________________________________________')
    disp('#######################################################################')   
    end  
    
    %% PLOTS DO ROBOT:
    
    %% POSIÇÃO HOME
    if select == 2
        disp('______________________________________________________________________')
        disp(' ')
        disp('Posição Home')
        disp('______________________________________________________________________')
        disp(' ')
        
        q_home = [0 0 0 0 0]; % valor das juntas em "home"
 
        T0_1 = eval(subs(Ti(:,:,1), q_home(1,1))); 
        T1_2 = eval(subs(Ti(:,:,2), q_home(1,2))); 
        T2_3 = eval(subs(Ti(:,:,3), q_home(1,3))); 
        T3_4 = eval(subs(Ti(:,:,4), q_home(1,4)));
        T4_5 = eval(subs(Ti(:,:,5), q_home(1,5)));

        % Representação dos Elos do workspace (mundo)
        T0_2 = T0_1 * T1_2;
        T0_3 = T0_1 * T1_2 * T2_3;
        T0_4 = T0_1 * T1_2 * T2_3 * T3_4;
        T0_5 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5;

        % Visualização
        axis = [-5 8 -2 8 -2 5];

        figure('units','normalized','outerposition',[0 0 1 1]);
        % Referencial Home 
        trplot(T0, 'axis', axis, 'color', 'r', 'frame', 'T0', 'view', [60 25]);
        hold on
        trplot(T0_1, 'axis', axis, 'color', 'g', 'frame', 'T1');
        hold on
        trplot(T0_2, 'axis', axis, 'color', 'b', 'frame', 'T2');
        hold on
        trplot(T0_3, 'axis', axis, 'color', 'm', 'frame', 'T3');
        hold on
        trplot(T0_4, 'axis', axis, 'color', 'k', 'frame', 'T4');
        hold on
        trplot(T0_5, 'axis', axis, 'color', 'c', 'frame', 'T5');

    disp('#######################################################################') 
    end 
    
    %% Animação do Robô da juntas Home para as juntas  objectivo
    if select == 3
        disp('______________________________________________________________________')
        disp(' ')
        disp('Animação')
        disp('______________________________________________________________________')
        disp(' ')
        
        k = pi/2;

        figure('units','normalized','outerposition',[0 0 1 1]);

        for i=0:0.1:k 

            hold off
            pause(0.0001)

            q = [ i -i -i i];

            animateRobot(T0, Ti, q);

        end

    disp('#######################################################################') 
    end 


end % fim do menu/ fim do exercicio


