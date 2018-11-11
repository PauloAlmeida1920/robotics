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

disp('**************************** ROBOT2 ***********************************')

syms theta1 d2 theta3 theta4 theta5

% Offset/comprimentos dos elos (fixos)
syms L1 L2 L5

% Junta Rotacional ou Prismatica:
R = 1; P = 0;

% Robot 1 [PPRRR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
%____________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%____________________________________________________________________________________
PJ_DH = [  theta1      0     L1    -pi/2     -pi/2           R;    % Junta prismática
%____________________________________________________________________________________
                0     d2      0        0        L2           P;    % Junta rotacional
%____________________________________________________________________________________
           theta3      0      0    -pi/2         0           R;    % Punho esférico
%____________________________________________________________________________________
           theta4      0      0     pi/2         0           R;    % Punho esférico
%____________________________________________________________________________________
           theta5     L5      0        0         0           R ];  % Punho esférico
%____________________________________________________________________________________

% A cinemática directa da base até ao Gripper:
[ T0_G, Ti ] = MGD_DH(PJ_DH);

% Offset/comprimentos dos elos (fixos)
PJ_DH =  eval(subs(PJ_DH, [L1 L2 L5], [1 0.5 0.25]));


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

%Valores Juntas "HOME"
q_home = [0 0 0 0 0];

% Valores Juntas aleatório
q = [deg2rad(35) 1 deg2rad(85) deg2rad(50) deg2rad(20)];


%% a) Representação do Gripper no mundo e b) Confirmação dos dados

T0_G_ =  eval(subs(T0_G, [L1 L2 L5], [1 0.5 0.25]));

% Matriz de transformação de O T G dados os valores das juntas do robô
T0_G_values = eval(subs(T0_G_, [theta1 d2 theta3 theta4 theta5], q));

% Confirmação da Matriz usando a robotics toolbox
T0_G_bytoolbox = robot.fkine(q);



%% c) Modelo inverso dos Robots e d) confirmação usando a robotics toolbox

% Atenção: Correr secção a) e b) - depende de T0_G_values e Ti

syms nx ny nz sx sy sz ax ay az tx ty tz

T0_G_nsat = [ nx sx ax tx;
              ny sy ay ty;
              nz sz az tz;
               0  0  0  1 ];

% Cinemática Inversa das juntas do Braço: d1 e d2

t0_G = T0_G(1:3,4);
Rz0_G = T0_G(1:3,3);

% 0 t 2 = 0 t G - L5 * 0 Rz G
t0_2 = simplify( t0_G - L5 * Rz0_G );

% tx' e ty'
tx_l = t0_2(1);
ty_l = t0_2(2);

% Vector Simbólico
t0_G = T0_G_nsat(1:3,4);
Rz0_G = T0_G_nsat(1:3,3);
t0_2_syms = simplify( t0_G - L5 * Rz0_G );


% Cinemática Inversa das juntas do Punho Esférico: theta3 theta4 theta 5

% Auxiliar Base ao Braço -> 2 T 0:
T0_1 = Ti(:,:,1);
T1_2 = Ti(:,:,2);

T0_2 = simplify( T0_1 * T1_2 );

T2_0 = simplify( inv(T0_2) );

% Matriz Simbólica 2 T G: De forma a conhecer que valores usar dados em O T G
T2_G_nsat = simplify( T2_0 * T0_G_nsat );

% Resultado - Gripper no Elo 2 -> 2 T G:
T2_G = simplify(  T2_0 * T0_G );


% Juntas do Robô dadas pela Cinemática Inversa do robot
q_byinv = inverse_kinematics_robot2(T0_G_values);

% Confirmação usando a robotics toolbox
q_bytoolbox = robot.ikine(T0_G_values, 'mask', [0 1 1 1 1 1]); % [x y z roll pitch yaw]



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
        disp('______________________________________________________________________')

        first = first + 1;
    end

    select = menu('Seleccione a acção a realizar:', 'a) & b) Plot do Robô',...
                                                    'a) & b) Confirmação',...
                                                    'c) & d) Cinemática Inversa',...
                                                    'Sair');

    % a) e b) Representação grafica dos robots c/ o punho esférico
    if select == 1
        close all;

        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de lado do Robot
        robot.teach(q_home, 'workspace', [-3 3 -3 3 -1 3], 'reach', 1,...
                            'scale', 1, 'zoom', 0.65, 'jaxes');
    end

    % a) e b) Confirmação dos dados
    if select == 2
        close all;
        disp('______________________________________________________________________')
        disp(' ')
        disp('Matriz transformação obtida com MGD_DH() através dos valores atribuidos em q[]')
        disp('______________________________________________________________________')
        disp(' ')
        disp('O T G:')
        disp(' ')
        disp(T0_G_values)
        disp('c) Confirmação usando a toolbox Robotics:')
        disp(' ')
        disp(T0_G_bytoolbox)
        disp('______________________________________________________________________')
    end

    % c) Modelo inverso dos Robots
    if select == 3
        close all;
        disp('______________________________________________________________________')
        disp(' ')
        disp('d) Solução de Cinemática Inversa')
        disp('______________________________________________________________________')
        disp(' ')
        disp('Matriz Simbólica -> O T G:')
        disp(' ')
        disp(T0_G_nsat)
        disp('Gripper na Base -> O T G:')
        disp(' ')
        disp(T0_G)

        disp('______________________________________________________________________')
        disp('Cinemática Inversa das juntas do Braço: theta1 e d2')
        disp(' ')
        fprintf("tx' = %s \n", t0_2_syms(1));
        fprintf("ty' = %s \n\n", t0_2_syms(2));        
        fprintf("tx' = %s \n", tx_l);
        fprintf("ty' = %s \n", ty_l);
        disp(' ')

        disp('______________________________________________________________________')
        disp('Cinemática Inversa das juntas do Punho Esférico: theta3 theta4 theta5')
        disp(' ')
        disp('Base ao Elo 2 -> 2 T 0:')
        disp(' ')
        disp(T2_0)
        disp('Matriz Simbólica - Gripper no Elo 2 -> 2 T G:')
        disp(' ')
        disp(T2_G_nsat)
        disp('Resultado - Gripper no Elo 2 -> 2 T G:')
        disp(' ')
        disp(T2_G)

        disp('______________________________________________________________________')
        disp(' ')
        disp('Valores das Juntas do robô usando o modelo da cinemática inversa')
        disp('______________________________________________________________________')
        disp(' ')
        disp(['q = [ ' num2str(rad2deg(q_byinv(1))) 'º ' ...
                       num2str(q_byinv(2)) 'm ' ...
                       num2str(rad2deg(q_byinv(3))) 'º ' ...
                       num2str(rad2deg(q_byinv(4))) 'º ' ...
                       num2str(rad2deg(q_byinv(5))) 'º ]'])
        disp(' ')
        disp('c) Confirmação usando a toolbox Robotics:')
        disp(' ')
        disp(['q = [ ' num2str(rad2deg(q_bytoolbox(1))) 'º ' ...
                       num2str(q_bytoolbox(2)) 'm ' ...
                       num2str(rad2deg(q_bytoolbox(3))) 'º ' ...
                       num2str(rad2deg(q_bytoolbox(4))) 'º ' ...
                       num2str(rad2deg(q_bytoolbox(5))) 'º ]'])
        disp(' ')
        disp('______________________________________________________________________')
    end

    % clear workspace
    if select == sair
       close all;
    end

end     %fim do menu
