clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%     [Robótica - 11/11/2018 ~ 2/12/2018] LABWORK#3 - PROBLEMA 2    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')

%% Robot RPR
disp('*************************** Exercício 2 ******************************')
disp(' ')
disp('Aguarde...')

% 1) Modelo cinemático directo do Robot RPR
syms theta1 d2 theta3 alfa

% Offset/comprimentos dos elos (fixos)
syms L4

% Junta Rotacional ou Prismática:
R = 1; P = 0;

% Robot [RPR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
%_________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%_________________________________________________________________________________
PJ_DH = [  theta1      0      0     pi/2     pi/2           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     d2      0    -pi/2        0           P;   % Junta Prismática
%_________________________________________________________________________________
           theta3      0      0     pi/2        0           R;   % Junta Rotacional
%_________________________________________________________________________________
                0     L4      0        0        0           R; ]; % (Não aplicável)
%_________________________________________________________________________________

% A cinematica directa da base até ao Gripper: 
[ T0_G, Ti ] = MGD_DH(PJ_DH);       

% Offset/comprimentos dos elos (fixos)
PJ_DH = eval(subs(PJ_DH, L4, 10));


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

% Inicialização do vector de juntas na nossa Posição "home"

% Posição HOME:
Tb_f = [ -cos(alfa) 0  sin(alfa)  40;
          sin(alfa) 0  cos(alfa)  20;
                  0 1          0   0;
                  0 0          0   1  ];

     
%% 2) Cinemática Inversa/Solução para as variáveis das juntas: theta1 d2 theta3

% NOTA: VER PDF EX2 COM SOLUÇÕES ANALÍTICAS PARA O CÁLCULO DA INVERSA

% Alfa em função do movimento circular
alfa_ = 0;
k = 1;
while( alfa_ < 2*pi)

    q(k,:) = inverse_kinematics_ex2(Tb_f, alfa_);
    
    % Incrementa um 1º
    alfa_ = alfa_ + pi/180;
    
    k = k + 1;
end
q = eval(q);


%% 3) Jacobiano: expressões para a velocidade de rotação das juntas 

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 d2 theta3 ];

% Construir jacobiana 2 partir dos parâmetros calculados na Cinemática inversa
Jac = Jacobian(T0_G, Ti, q_aux, PJ_DH(:,6));

% Componentes de velocidade objectivo [ vx vy wz ]
Jac_ = [ Jac(1:2, 1:3); Jac(6,1:3) ];

% Restrição na velocidade Wz
Wz = pi;

% Inversa da Jacobiana x Velocidades em
qVelocidades = inv(Jac_)*[ 0 0 Wz ]';
        

%% 4) Movimento do manipulador: MAIS A BAIXO NO CÓDIGO DO MENU



%% MENU ("main")

% Variaveis MENU
select = 0;
select2 = 0;
STOP = 5;
STOP2 = 3;
first = 0;
fix_bug = 0;
    
while(select ~= STOP)
    
    select = menu('Seleccione:', 'Plot do Robô',...
                                 'alínea 2)',...
                                 'alínea 3)',...
                                 'alínea 4)',...
                                 'Quit');  
                                         
   % Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH e a O T G
    if first < 1
        disp('______________________________________________________________________')
        disp(' ')
        disp('Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH')
        disp('______________________________________________________________________')
        disp(' ')
        robot.display
        disp('______________________________________________________________________')

        first = first + 1;
    end  
    
    % PLOT DO ROBOT:
    
    if select == 1
        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de lado do Robot  
%         subplot(1,2,1);

        robot.plot([q(1,:) 0], 'workspace', [-10 90 -10 90 -10 20], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25, 'jaxes');
                       
%         % Prespectiva de topo do Robot  
%         subplot(1,2,2);
%         robot.plot(q(1,:), 'workspace', [-10 90 -10 90 -10 20], 'reach', ... 
%                        1, 'scale', 10, 'zoom', 0.25, 'view', 'top', 'jaxes');
                  
    end
    
    % 2) Movimento circular em função da cinemática inversa
    if select == 2        
        figure('units','normalized','outerposition',[0 0 1 1]);       
        % Prespectiva de topo do Robô  
        robot.plot([q zeros(size(q,1),1)], 'workspace', [-10 70 -20 40 -1 10], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25, 'view', 'top', 'jaxes');
      
    disp('______________________________________________________________________')    
    end % fim da alinea 2)
    
    % 3) Calcule as ExpressÃµes para a velocidade das juntas
    if select == 3        
        disp('______________________________________________________________________')
        disp(' ')
        disp('b) Jacobiano:')
        disp('______________________________________________________________________')
        disp(' ')
        disp('Expressões para a velocidade das juntas c/ Wz = pi rad/s')
        disp(' ')
        w1 = qVelocidades(1)
        vd = qVelocidades(2)
        w3 = qVelocidades(3)
        
    disp('______________________________________________________________________')    
    end % fim da alinea 3)
    
    % 4) Movimento do manipulador com malha de controlo
    if select == 4
       % sub-menu
       while(select2 ~= STOP2)
           
           select2 = menu('Seleccione: ', 'Integrador',...
                                          'Malha-Fechada',...
                                          'Back');
           %---------------------------------------------------------------        
           
           % Velocidades impostas: Velocidade angular Wz
           Wz = pi;
                                         
           % Período de Amostragem dos Controladores 
           h = 0.1;
           
           % Inicializa as Juntas segundo a Matriz Home/Posição Inicial
           alfa_ = 0;
           
           if fix_bug ~= 1
                Tb_f = eval(subs(Tb_f, alfa, alfa_));
                fix_bug = 1;
           end
       
           [ q_controlo ] = inverse_kinematics_ex2(Tb_f, alfa_);
           
           Jac_ = eval(subs(Jac_, L4, 10));
           T0_G = eval(subs(T0_G, L4, 10));
                   
           %»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»
           % 1. Abordagem Integradora
           if select2 == 1
               
               k = 1;
               
               while(k < 30) 
                   
                   % [ Vx Vy Wz ]
                   V(k,:) = [ 0 0 Wz ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ
                   qVelocidades_ = inv(Jac_)*V(k,:)';
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux,...
                                                 q_controlo(k,:) ));
                   
                   % Proximas Juntas segundo a Lei de Controlo: Abordagem Integradora
                   q_controlo(k+1,:) = q_controlo(k,:) + h*qVelocidades(:,k)';
                   
                   
                   % tx e ty através da Matriz da Cinemática Directa
                   Tb_f(:,:,k+1) = eval(subs(T0_G, q_aux,...
                                                   q_controlo(k,:) ));
                                         
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/29)*100), '%'])
                   
                   % Atendendo que a junta correspondente ao gripper é fixa
                   % acrescentamos 0 à ultima junta de forma a trabalharmos na Toolbox 
                   q_out(k,:) = [ q_controlo(k,1:3) 0 ];
                   pos_out(k,:) = Tb_f(1:3,4,k);
                   
                   k = k + 1;
               end
               
               % PLOT do Robot com velocidades
               plot_robot2(robot, k, V, qVelocidades, q_out, pos_out);
               
           end
           
           
           %»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»           
           % 2. Abordagem em malha-fechada
           if select2 == 2
               
               % Controlo Propocional
               kp = 0.25;
               
               % PerÃ­odo de Amostragem do Controlador
               h = 0.01;
               
               % Inicializa alfa desejado
               alfa_desej = alfa_;
               
               % Deslocamento de alfa
               dalfa = -Wz*h;
               
               % Velocidade Angular Desejada em Z
               Wz_desej = pi; 
               % Velocidade Angular Inicial
               Wz = 0;
               
               k = 1;
               
               while(k < 54)
                   
                   % [ Vx Vy Wz ]
                   V(k,:) = [ 0 0 Wz ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ
                   qVelocidades_ = inv(Jac_)*V(k,:)';
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux,...
                                                        q_controlo(k,:) ));
                   
                   % Proximas Juntas: Controlo Malha fechada
                   q_controlo(k+1,:) = q_controlo(k,:) + 1*h*qVelocidades(:,k)';
                   
                 
                   % tx e ty através da Matriz da Cinemática Directa
                   Tb_f(:,:,k+1) = eval(subs(T0_G, q_aux,...
                                                   q_controlo(k,:) ));
                   
                   % ------------------------------------------------------
                   % Clacula a alfa actual     
                   alfa_actual = acot(tan(q_controlo(k+1,1) + q_controlo(k+1,3)));
                   alfa_actual = wrapTo2Pi(alfa_actual);
                                   
                   % Alfa desejado 
                   alfa_desej = alfa_desej + dalfa;
                   alfa_desej = wrapTo2Pi(alfa_desej);
                   
                   % Erro da Velocidade angular Wz
                   alfa_erro(k) = alfa_desej - alfa_actual;
                   
                   % Velocidade actual
                   Wz_actual(k) = alfa_erro(k)/h;
                   
                   Wz_erro(k) = Wz_desej - Wz_actual(k);
                 
                   Wz = kp*Wz_erro(k);
                     
                                   
                   % ------------------------------------------------------        
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str((k/53)*100), '%'])
                  
                   % Atendendo que a junta correspondente ao gripper é fixa
                   % acrescentamos 0 a ultima junta de forma a trabalharmos na Toolbox 
                   q_out(k,:) = [ q_controlo(k,1:3) 0 ];
                   pos_out(k,:) = Tb_f(1:3,4,k);
                   
                   k = k + 1;
               end
               
               % PLOT do Robot com velocidades
               plot_robot2(robot, k, V, qVelocidades, q_out, pos_out);
 
           end
           % --------------------------------------------------------------         
       end % fim do sub-menu
      select2 = 0;
      
    disp('______________________________________________________________________')   
    end % fim da alinea 4)   
end % fim do menu/ fim do exercicio









