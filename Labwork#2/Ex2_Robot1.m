clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robótica - 23/10/2018 ~ 11/11/2018] LABWORK#2 - PROBLEMA 2    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')

%% Problema 2 - Obtenha os parametros de D-H dos 3 manipuladores

disp('**************************** ROBOT1 ***********************************')

syms d1 d2 theta3 theta4 theta5

L1 = 0.25;

% Junta Rotacional ou Prismatica:
R = 1; P = 0;

% Robot 1 [PPRRR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
%____________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%____________________________________________________________________________________
PJ_DH = [       0     d1      0    -pi/2         0           P;    % Junta prismática
%____________________________________________________________________________________
             pi/2     d2      0        0         0           P;    % Junta prismática
%____________________________________________________________________________________
            theta3     0      0    -pi/2         0           R;    % Punho esferico
%____________________________________________________________________________________
            theta4     0      0     pi/2         0           R;    % Punho esferico
%____________________________________________________________________________________
            theta5    L1      0        0     -pi/2           R ];  % Punho esferico
%____________________________________________________________________________________


%% INICIALIZAÇÃO DO ROBOT: CRIAR LINKS

for i = 1 : size(PJ_DH,1)
    
    if PJ_DH(i,6) == R              % Juntas Rotacionais
        
        L(i) = Link('d',eval(PJ_DH(i,2)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)));
    end
    
    if PJ_DH(i,6) == P              % Junta Prismática
        
        L(i) = Link('theta',eval(PJ_DH(i,1)),...
                    'a', eval(PJ_DH(i,3)),...
                    'alpha', eval(PJ_DH(i,4)),...
                    'offset', eval(PJ_DH(i,5)),...
                    'qlim', [0 1]);
    end

end

robot = SerialLink(L, 'name', 'Robô Planar PPRRR');


%% VARIÁVEIS GLOBAIS

% Valores Juntas - "home"
q = [1 1 0 0 0];

%q = [0 0 0 0 0];
        

%% a) e b) Confirmação dos dados

% A cinemática directa da base   até ao Gripper: 
[ T0_G, Ti ] = MGD_DH(PJ_DH);   

% Matriz de transformação de O T G dados os valores das juntas do robô
T0_G_values = eval(subs(T0_G, [d1 d2 theta3 theta4 theta5], q));

% Confirmação da Matriz usando a robotics toolbox 
T0_G_bytoolbox = robot.fkine(q);


%% c) Modelo inverso dos Robots e confirmação usando a robotics toolbox 

% Atenção: Correr secção a) e b) - depende de T0_G_values

% Juntas do Robô dadas pela Cinemática Inversa do robot
q_byinv = inverse_kinematics_robot1(T0_G_values);


% Confirmação usando a robotics toolbox 
q_bytoolbox = robot.ikine(T0_G_values, 'mask', [0 1 1 1 1 1]); % [x y x roll pitch yaw] 



%% MENU ("main")

select = 0;
first = 0;
sair = 4;

while (select ~= sair)
    
    % Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH e a O T G
    if first < 1
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH')
        disp('______________________________________________________________________')
        disp(' ')
        robot.display
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Cinemática Directa: O T G')
        disp('______________________________________________________________________')
        disp(' ')
        disp(T0_G)
        disp(' ')
        disp('______________________________________________________________________')
        
        first = first + 1;
    end
    
    select = menu('Seleccione a acção a realizar:', 'a) & b) Plot do Robô',...
                                                    'a) & b) Confirmação',...
                                                    'c) & d) Cinemática Inversa',...
                                                    'Sair');
                                                
    % a) & b) Representação grafica dos robots c/ o punho esférico
    if select == 1
        
        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de lado do Robot  
        robot.teach(q, 'workspace', [-2 2 -2 2 -1 2], 'reach', ... 
                       1, 'scale', 1, 'zoom', 1);
    end
    
    % a) e b) Confirmação dos dados
    if select == 2     
        disp('Matriz de transformação para cinematica Directa obtida com MGD_DH() c/ variaveis simbolicas:')
        disp('____________________________________________________________________________________')
        T0_H = simplify(T0_H);
        disp(T0_H)
        disp('____________________________________________________________________________________')

        disp(' ')
        disp('Matriz transformação obtida com MGD_DH() e com os valores atribuidos em q[]')
        disp('____________________________________________________________________________________')
        disp(T0_H_values)
        disp('____________________________________________________________________________________')

        disp(' ')
        disp('Matriz de transformação obtida com a toolbox e com os valores artribuidos em q[]')
        disp('____________________________________________________________________________________')
        disp(T0_H_Toolbox)
        disp('____________________________________________________________________________________')

        if(abs((T0_H_values - T0_H_Toolbox)) < 1e-15)
            disp('Os valores obtidos são coincidentes com os obtidos a partir da toolbox');
        else
            disp('As matrizes não coincidem!');
        end
    end
    
    % c) Modelo inverso dos Robots
    if select == 3
        if(abs((T0_H_values_inv - T0_H_Toolbox_inv)) < 1e-15)
            disp('Os valores obtidos são coincidentes com os obtidos a partir da toolbox');
        else
            disp('As matrizes não coincidem!');
        end
    end
    
    % clear workspace
    if select == sair
       close all; 
    end
    
end     %fim do menu