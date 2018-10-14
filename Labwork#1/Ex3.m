clear all
close all
clc

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%                                                                  %%')
disp('%%          [Robótica - 07/11/2017 ~ 26/11/2017] LABWORK#3          %%')
disp('%%                                                                  %%')
disp('%%                   Frederico Vaz, nº 2011283029                   %%')
disp('%%                   Paulo Almeida, nº 2010128473                   %%')
disp('%%                                                                  %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')
disp('*************************** Exercício 3 ******************************')

%% Robot 3-DOF (2 de rotação e 1 prismáticos): RPR

syms theta1 d2 theta3 alfa

% Comprimentos dos elos:
L2 = 10;

% Junta Rotacional ou Prismatica:
R = 1; P = 0;
%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1      0      0     pi/2     pi/2           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta2      0      0    -pi/2        0           P;   % Junta Rotacional
%_________________________________________________________________________________
           theta3      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
           theta4      0      0    -pi/2    -pi/2           R; % Junta Rotacional
%_________________________________________________________________________________
           theta5      1      0        0        0           R; ]; % Junta Rotacional
%_________________________________________________________________________________

% A cinematica directa da base   até ao Gripper: 
[ oTg, Ti ] = direct_kinematics(PJ_DH);       

oTg = simplify(oTg);
Ti  = simplify(Ti) ;


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
                    'qlim', [25 50]);
        
    end

end

robot = SerialLink(L, 'name', 'Robot Planar RRR');


%% VARIÁVEIS GLOBAIS 

% Inicialização do vector de juntas na nossa posição "home" a começar no
% no ponto inicial [ bmin, -30, 35] dado no enuciado 

% POSIÇÃO HOME:
bTf = [  0  -cos(alfa)  sin(alfa)  40;
         0   sin(alfa)  cos(alfa)  20; 
         0   0          0           0;  
         0   0          0           1  ];

[ q ] = inverse_kinematics_ex3(bTf, 0);

q = eval([ q(1:3) 0 ]);

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 d2 theta3 ];

% Construir jacobiana 2 partir dos parâmetros calculados na cinemática inversa
Jac = Jacobian(oTg, Ti, q_aux, PJ_DH(:,6));

% Componentes de velocidade objectivo [ vx vy wz ]
Jac_ = [ Jac(1:2, 1:3); Jac(6,1:3) ];


%% MENU ("main")

% Variaveis MENU
select = 0;
STOP = 2;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'Cinematica Directa',...
                                                   'Plot do Robot',...
                                                   'Quit');  
                                                
    %% Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('PJ_DH: Matriz dos parametros de Denavith-Hartenberg:')
        disp('______________________________________________________________________')
        disp(' ')
        PJ_DH_ = SerialLink(L, 'name', 'Robot Planar RRPRP')
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
    
    %% PLOT DO ROBOT:
    
    if select == 2
        figure('units','normalized','outerposition',[0 0 1 1]);
         % Prespectiva de lado do Robot  
        subplot(1,2,1);
        robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25); % 'view', 'top', 'trail', 'b.');
                   
        % Prespectiva de topo do Robot -------------------------------------
        subplot(1,2,2);
        robot.plot(q, 'workspace', [-10 90 -10 90 -10 90],...
                      'reach', 1,...
                      'scale', 10,...
                      'zoom', 0.25,...
                      'view',...
                      'top'); % 'trail', 'b.');
                   
    disp('#######################################################################') 
    end  


end % fim do menu/ fim do exercicio

