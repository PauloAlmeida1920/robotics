clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%     [Robótica - 11/11/2018 ~ 2/12/2018] LABWORK#3 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')

%% Robô num processo de	pintura
disp('*************************** Exercício 1 ******************************')

syms theta1 theta2 theta3

% Offset/comprimentos dos elos (fixos)
syms L1 L2 L3
% NOTA: L1, L2 <=> a2 & L3 <=> d3

% Junta Rotacional ou Prismatica:
R = 1; P = 0;

% Robot 1 [RRR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
%____________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%____________________________________________________________________________________
PJ_DH = [  theta1      0     L1        0         0           R;   % Junta Rotacional
%____________________________________________________________________________________    
           theta2      0     L2        0         0           R;   % Junta Rotacional
%____________________________________________________________________________________           
           theta3     L3      0        0         0           R ]; % Junta Rotacional + Gripper
%____________________________________________________________________________________

% A cinematica directa ate o Gripper 
[ T0_G, Ti ] = MGD_DH(PJ_DH);       

% Offset/comprimentos dos elos (fixos)
PJ_DH = eval(subs(PJ_DH, [L1 L2 L3], [35 35 10])); % a2 = 40 ou 35??


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

% Inicialização do vector de juntas
q = [ 0 0 0 ];

% Posição Home
Tb_f = [ 0  -1   0   20; % 20
         1   0   0   50;
         0   0   1   10;
         0   0   0   0  ];


%% a) Cinemática Inversa/Solução para às variáveis das juntas: theta1 theta2 theta3

%NOTA: VER PDF EX1, SOLUÇÕES AQUI PARA COMPLEMENTAR CÁCULOS

syms nx ny nz sx sy sz ax ay az tx ty tz alpha
     
% Vector t           
tx_ = T0_G(1,4);
ty_ = T0_G(2,4);     

    
% 1ª Solução: theta3
equation1 = alpha == theta1 + theta2 + theta3;

% Pela equação facilmente extraímos o theta3:
theta3_sol = solve(equation1, theta3);

theta3_sol = subs(theta3_sol, alpha, atan2( ay, ax ));


% 2ª Solução: theta2

% Resolve a equação  tx² + ty² = tx_² + ty_²
equation2 = tx^2 + ty^2 == simplify( tx_^2 + ty_^2 );

% Pela equação facilmente extraímos o theta2:
theta2_sol = simplify( solve(equation2, theta2) );


% 3ª Solução: theta1

%NOTA: VER PDF EX1 PARA ESTA SOLUÇÃO

% Substitui a razão trigonométrica c12 = c1*c2 - s1*s2
tx__ = expand(subs(tx_, cos(theta1 + theta2), cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)));
ty__ = expand(subs(ty_, cos(theta1 + theta2), cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2)));

% Coloca em ordem a c1 e c2
tx__ = collect(tx__, cos(theta1)) * (1/((L1 + L2*cos(theta2))*cos(theta1)));
ty__ = collect(ty__, sin(theta1)) * (1/((L1 + L2*cos(theta2))*cos(theta1)));

% Coloca em ordem s1/c1
tx__ = collect(tx__, sin(theta1)/cos(theta1));
ty__ = collect(ty__, sin(theta1)/cos(theta1));

% Resolve a equação ty / tx = tx_ / ty_ <=> alpha = atan( ty, tx ):
equation = ty/tx ==  simplify( (tx__) / (ty__) );

% Fica a c1:
equation3 = isolate(equation, cos(theta1));

%NOTA: VER PDF EX1 PARA ESTA SOLUÇÃO


% Juntas do Robô dadas pela Cinemática Inversa do robot
q_byinv = inverse_kinematics_ex1(Tb_f);

% Confirmação usando a robotics toolbox
%q_bytoolbox = robot.ikine(Tb_f, 'mask', [1 1 0 0 0 1]) % [x y z roll pitch yaw]


%% b) Jacobiano: expressões para a velocidade de rotação das juntas 

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 theta2 theta3 ];
        
% Construir a Jacobiana a partir dos parâmetros calculados na cinemática inversa
Jac = Jacobian(T0_G, Ti, q_aux, PJ_DH(:,6));

% Retirar as componentes de velocidade nula [ vz wx wy ]
Jac_ = [ Jac(1:2,:); Jac(6,:) ];
        
% Restrição na velocidade em cm/s
Vx = sqrt(5)*3;
Vy = sqrt(5)*6;

% Inversa da Jacobiana x Velocidades em XYZ
qVelocidades = inv(Jac_)*[ Vx Vy 0 ]';


%% c) Movimento do manipulador: MAIS A BAIXO NO CÓDIGO DO MENU

        
%% MENU ("main")

select = 0;
select2 = 0;
STOP = 5;
STOP2 = 3;
first = 0;

while(select ~= STOP)
    
    select = menu('Seleccione:', 'Plot do Robô',...
                                 'alínea a)',...
                                 'alínea b)',...
                                 'alínea c)',...
                                 'Sair');    
    
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
        
        % Prespectiva de lado do Robô  
%         subplot(1,2,1);
        robot.plot(q, 'workspace', [-10 95 -10 95 -10 30], 'reach', ... 
                       1, 'scale', 10, 'zoom', 0.25, 'jaxes');         
%         % Prespectiva de topo do Robô  
%         subplot(1,2,2);
%         robot.plot(q, 'workspace', [-10 90 -10 90 -10 90], 'reach', ... 
%                        1, 'scale', 10, 'zoom', 0.25, 'view', 'top', 'jaxes');
                  
    end   
    
    % a) Solucão para as variáveis	das	juntas do robot	q = [ theta1 theta1 theta3 ] que garanta o 
    %    correto posicionamento da ferramenta na posição "home" do processo de pintura.
    if select == 2
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Robô na posição home:')
        disp('______________________________________________________________________')

        [ q ] = inverse_kinematics_ex1(Tb_f);

        disp(' ')
        disp(['Segundo ao vector q = [ ' num2str(rad2deg(q(1,1))) 'º ' num2str(rad2deg(q(1,2))) 'º '...
               num2str(rad2deg(q(1,3))) 'º ] extraído através da cinemática inversa.'])

        % Plot do Robo na posição home!
        disp(' ')
        disp('Clique em "Plot do Robô" para o visualizar!')
    
    disp(' ')
    disp('______________________________________________________________________')
    end % fim da alinea a)    
    
    % b) Jacobiano - Expressões para a velocidade de rotação das juntas 
    %    que asseguram a restrição temporal de pintura (v=15cm/seg).	
    if select == 3
        disp('______________________________________________________________________')
        disp(' ')
        disp('b) Jacobiano:')
        disp('______________________________________________________________________')
        disp(' ')
        disp('Expresoes para a velocidade das juntas p/ V = 15cm/s:')
        disp(' ')
        disp(qVelocidades)
     
    disp('______________________________________________________________________')   
    end % fim da alinea b)
    
    % c) Movimento do manipulador com malha de controlo
    if select == 4
       % sub-menu
       while(select2 ~= STOP2)
           
           select2 = menu('Seleccione:', 'Integrador',...
                                         'Malha-Fechada',...
                                         'Back'); 
           %---------------------------------------------------------------
           
           % Restrição imposta no enuciado (e têm que ser negativo
           % para ir no sentido da base)
           Vx = -sqrt(5)*3;
           % Posição Y segundo a equação da reta y = 2x + 10
           Vy = -sqrt(5)*6; %2*Vx + 10;
           
           % Posição Final do Gripper
           Xf = -20; Yf = -50;
                              
           % Período de Amostragem dos Controladores 
           h = 0.15;
           
           % Inicializa as Juntas segundo a Matriz Home/Posição Inicial
           [ q_controlo ] = inverse_kinematics_ex1(Tb_f);
           
           Jac_ = eval(subs(Jac_, [L1 L2 L3], [40 40 10]));
           T0_G = eval(subs(T0_G, [L1 L2 L3], [40 40 10]));
           
           % 1. Abordagem Integradora (Malha Aberta)
           if select2 == 1
              
               k = 1;
                
               while(Tb_f(1,4,k) > Xf || Tb_f(2,4,k) > Yf)
                    
                   % [ Vx Vy Vz ]
                   V(k,:) = [ Vx Vy 0 ];
                   
                   % Inversa do Jacobiano x Velocidades em XYZ
                   qVelocidades_ = inv(Jac_)*V(k,:)';
                   
                   
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux, q_controlo(k,:)));
                   
                   % Proximas Juntas segundo a Lei de Controlo: Abordagem Integradora
                   q_controlo(k+1,:) = q_controlo(k,:) + h*qVelocidades(:,k)';
                   
                   
                   % tx e ty através da Matriz da Cinemática Directa
                   Tb_f(:,:,k+1) = eval(subs(T0_G, q_aux, q_controlo(k+1,:)));
                   
                   
                   pos_out(k,:) = Tb_f(1:3,4,k);
                   
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str(k/50*100), '%']) 
                    
                   k = k + 1;
               end
               
               % PLOT do Robô com velocidades
               plot_robot(robot, k, V, qVelocidades, q_controlo, pos_out);
              
           end
           %---------------------------------------------------------------
           
           % 2. Abordagem em malha-fechada
           if select2 == 2

               % Controlo Propocional
               kp = 0.25;
               % Controlo Derivativo
               kd = 0.1;
               
               % Posição Inicial!
               tx_desej = 20;
               ty_desej = 50;
               
               % Deslocamento = Velocidade*Período IDEAL!
               dx = Vx*h;
               dy = Vy*h;
               
               % Velocidade Inicial
               Vx = 0;
               Vy = 0;
               
               k = 1;
               
               while(Tb_f(1,4,k) > Xf || Tb_f(2,4, k) > Yf)

                   % [ Vx Vy Vz ]
                   V(k,:) = [ Vx Vy 0 ];
                 
                   % Inversa do Jacobiano x Velocidades em XYZ 
                   qVelocidades_ = inv(Jac_)*V(k,:)';  
                   % Calculo da Inversa do Jacobiano
                   qVelocidades(:,k) = eval(subs(qVelocidades_, q_aux, q_controlo(k,:)));
                   
                   % Proximas Juntas
                   q_controlo(k+1,:) = q_controlo(k,:) + h*qVelocidades(:,k)';
                   
                   % tx e ty através da Matriz da Cinemática Directa -> f(q(k))
                   Tb_f(:,:,k+1) = eval(subs(T0_G, q_aux, q_controlo(k+1,:)));
                   
                   % ------------------------------------------------------
                   % Posição actual
                   tx_actual = Tb_f(1,4,k);
                   ty_actual = Tb_f(2,4,k);
                  
                   % Posição desejada = Posição + Deslocamento IDEAL!
                   tx_desej = tx_desej + dx;
                   ty_desej = ty_desej + dy;
                   
                   dx_erro = tx_desej - tx_actual;
                   dy_erro = ty_desej - ty_actual;
                   
                   Vx = kp*dx_erro + kd*dx_erro/h; 
                   Vy = kp*dy_erro + kd*dy_erro/h;
                   
                   
                   % ------------------------------------------------------  
                   
                   pos_out(k,:) = Tb_f(1:3,4,k);
                   
                   clc
                   disp(' ')
                   disp(['Loading... ', num2str(k/57*100), '%']) 
                   
                   k = k + 1;
               end
               
               % PLOT do Robô com velocidades
               plot_robot(robot, k, V, qVelocidades, q_controlo, pos_out);          
             
           end
       end % fim do sub-menu
       select2 = 0;
        
    disp('______________________________________________________________________') 
    end % fim da alinea c)
    
end % fim do menu/ fim do exercicio