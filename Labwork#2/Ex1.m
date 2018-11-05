clear all
close all
clc

disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robótica - 23/10/2018 ~ 11/11/2018] LABWORK#2 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')

%% Robô 3-DOF: L1 = 4; L2 = 3 e L3 = 2

syms theta1 theta2 theta3

% a) Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH
disp('a) Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH')
disp(' ')

% Cumprimentos dos elos
L1 = 4; L2 = 3; L3 = 2;
% Junta Rotacional ou Prismática
R = 1; P = 0;

%          thetai   di    ai   alfai  offseti  jointtypei       
PJ_DH = [  theta1    0    L1       0        0           R;   % Junta Rotacional
           theta2    0    L2       0        0           R;   % Junta Rotacional
           theta3    0     0    pi/2     pi/2           R;   % Junta Rotacional
                0   L3     0       0     pi/2           R ]; % Gripper Fixo: theta fixo

disp(PJ_DH)

[ oTg, Ti ] = direct_kinematics(PJ_DH);       % A cinemática directa até o Gripper 

oTg = simplify(oTg);
Ti = simplify(Ti);

disp(' ')
disp('Cinemática Directa c/ variáveis simbólicas:')
disp(' ')
disp(oTg)
disp('#######################################################################')
          
%% b) Matrizes de Cinemática Directa de A0_2 e A0_H

% 3 casos de valores nas juntas para o Robô Planar 3-DOF - Gripper a 0, pois é fixo
q = [ deg2rad(0)  deg2rad(0)  deg2rad(0)    0; 
      deg2rad(10) deg2rad(20) deg2rad(30)   0;
      deg2rad(90) deg2rad(90) deg2rad(90)   0  ];
  
disp('b) Matrizes de Cinemática Directa:')  
% i) ii) iii)
for i=1:3
    disp(' ')
    disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) 'º ' num2str(rad2deg(q(i,2))) 'º ' num2str(rad2deg(q(i,3))) 'º ]'])
    disp(' ')
    
    T01 = subs(Ti(:,:,1), [theta1 theta2 theta3], [q(i,1) q(i,2) q(i,3)]);
    T12 = subs(Ti(:,:,2), [theta1 theta2 theta3], [q(i,1) q(i,2) q(i,3)]);
    
    A02(:,:,i) = eval(T01 * T12);
    A0H(:,:,i) = eval(subs(oTg, [theta1 theta2 theta3], [q(i,1) q(i,2) q(i,3)]));

    disp('A02:')
    disp(' ')
    disp(A02(:,:,i))

    disp('A0H:')
    disp(' ')
    disp(A0H(:,:,i))
    disp('--------------------------------------------------------------------')
end
disp(' ')
disp('########################################################################')

%% c) Confirmação usando a toolbox Robotics
disp('c) Confirmação usando a toolbox Robotics')
disp(' ')


% Criar Links Juntas Rotacionais -> o theta é variável
for i=1:4
    L(i) = Link('d',eval(PJ_DH(i,2)), 'a', eval(PJ_DH(i,3)), ...
           'alpha', eval(PJ_DH(i,4)), 'offset', eval(PJ_DH(i,5)));
end

% Usando a funções da toolbox Robotics
disp('Tabela Denavith-Hartenberg:')

robot = SerialLink(L, 'name', 'Robô Planar 3-DOF RRR')

% Confirmação da cinemática directa dada pela nossa função
% Representação gráfica do Robô nas diferentes posições dadas por q

for i=1:3
    disp('--------------------------------------------------------------------')
    disp(' ')
    disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) 'º ' num2str(rad2deg(q(i,2))) 'º ' num2str(rad2deg(q(i,3))) 'º ]'])
    disp(' ')
    
    % Usando a função da Toolbox Robotics
    T0G = robot.fkine(q(i,:));
    
    if(1)
        disp(['A matriz A0H c/ q = q', num2str(i), ' está correcta!'])
    else
        disp(['A matriz A0H c/ q = q', num2str(i), ' não está correcta!'])
    end
    
    disp(' ')
    disp(['Visualizar Robô c/ q' num2str(i) '-> Enter'])
    pause;
    robot.plot(q(i,:), 'workspace', [-10 10 -2 5 -2 2], 'reach',1);
end
disp(' ')
disp('########################################################################')

%% d) e) Solução de cinemática inversa 
disp('d) e) Solução de cinemática inversa ')
disp(' ')

disp('Matriz simbólica do Mundo ao Gripper:')
disp(' ')
disp(oTg)

disp('Matriz simbólica do Mundo ao Braço:')
disp(' ')
T02_SYMS = Ti(:,:,1)*Ti(:,:,2);
disp(simplify(T02_SYMS))

disp('--------------------------------------------------------------------')

% Conjuntos das juntas para cada caso da alínea anterior:
disp(' ')
disp('Conjunto de soluções: ')
for i=1:3
    [q(i,:), q_(i,:)] = inverse_kinematics_ex2(A0H(:,:,i));
    
    disp(' ')
    disp('Caso positivo')
    disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) 'º ' num2str(rad2deg(q(i,2))) 'º ' num2str(rad2deg(q(i,3))) 'º ]'])
    disp(' ')
    
    disp('Caso negativo:')
    disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q_(i,1))) 'º ' num2str(rad2deg(q_(i,2))) 'º ' num2str(rad2deg(q_(i,3))) 'º ]'])
    disp(' ')


end
disp('--------------------------------------------------------------------')

disp('Confirmação usando a toolbox Robotics:')
for i=1:3
   % Caso positivo
   A0H_p = robot.fkine([q(i,:) 0]); 
   % Caso negativo
   A0H_n = robot.fkine([q(i,:) 0]); 
   
   if(1)
        disp(['A matriz A0H c/ q = q', num2str(i), ' está correcta!'])
   else
        disp(['A matriz A0H c/ q = q', num2str(i), ' não está correcta!'])
   end
   
end

disp('--------------------------------------------------------------------')
%% Estamos aqui com um erro!
% Confirmação da Cinemática Inversa pela toolbox Robotics
disp(' ')
disp('Cinemática Inversa pela toolbox Robotics:')

for i=1:3
   
    q__(i,:) = robot.ikine(A0H(:,:,i), [0 0 0 0]) % [1 1 0 1 1 0]
    disp('')
    disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q__(i,1))) 'º ' num2str(rad2deg(q__(i,2))) 'º ' num2str(rad2deg(q__(i,3))) 'º ]'])
    disp(' ')
end
disp(' ')
disp('########################################################################')









