clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Rob�tica - 23/10/2018 ~ 11/11/2018] LABWORK#2 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, n� 2011283029                    %%')
disp('%%                   Paulo Almeida, n� 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')

%% Rob� 3-DOF: L1 = 4; L2 = 3 e L3 = 2

syms theta1 theta2 theta3

% a) Matriz dos par�metros de Denavith-Hartenberg: PJ_DH

% Cumprimentos dos elos
L1 = 4; L2 = 3; L3 = 2;
% Junta Rotacional ou Prism�tica
R = 1; P = 0;

%          thetai   di    ai   alfai  offseti  jointtypei       
PJ_DH = [  theta1    0    L1       0        0           R;   % Junta Rotacional
           theta2    0    L2       0        0           R;   % Junta Rotacional
           theta3    0     0    pi/2     pi/2           R;   % Junta Rotacional  
                0   L3     0       0     pi/2           R ]; % Gripper Fixo


% A cinem�tica directa at� ao Gripper
[ T0_G, Ti ] = MGD_DH(PJ_DH);  


% Criar Links Juntas Rotacionais -> o theta � vari�vel
for i=1:4
    L(i) = Link('d',eval(PJ_DH(i,2)), 'a', eval(PJ_DH(i,3)), ...
           'alpha', eval(PJ_DH(i,4)), 'offset', eval(PJ_DH(i,5)));
end

robot = SerialLink(L, 'name', 'Rob� Planar 3-DOF RRR');
robot_T0_2 = SerialLink(L(1:2));


%% b) Matrizes de Cinem�tica Directa de 0 A 2 e 0 A H;  c) Confirma��o

% 3 casos de valores nas juntas para o Rob� Planar 3-DOF - Gripper � fixo
q = [ deg2rad(0)  deg2rad(0)  deg2rad(0); 
      deg2rad(10) deg2rad(20) deg2rad(30);
      deg2rad(90) deg2rad(90) deg2rad(90) ];
  
% i) ii) iii)
for i=1:3
    
    T0_1 = eval(subs(Ti(:,:,1), q(i,1))); 
    T1_2 = eval(subs(Ti(:,:,2), q(i,2))); 
    T2_I = eval(subs(Ti(:,:,3), q(i,3))); 
    TI_H = Ti(:,:,4);    
   
    A0_2(:,:,i) = T0_1 * T1_2;
    A0_H(:,:,i) = eval( T0_1 * T1_2 * T2_I * TI_H );
    
    
    % c) Confirma��o das Matrizes usando a toolbox Robotics
    T02(:,:,i) = robot_T0_2.fkine(q(i,1:2));
    T0H(:,:,i) = robot.fkine([q(i,:) 0]);
    
end



%% d) e) Solu��o da Cinem�tica Inversa 

% Matriz simb�lica do Mundo ao Bra�o: O T 2
T0_1 = Ti(:,:,1);
T1_2 = Ti(:,:,2);

T0_2 = simplify( T0_1 * T1_2 );


% P/ os conjuntos das juntas da al�nea anterior:
for i=1:3
    
    [q(i,:), q_(i,:)] = inverse_kinematics_ex2(A0_H(:,:,i));
    
end

% Confirma��o usando a toolbox Robotics
for i=1:2

    q_ikine(i,:) = robot.ikine(A0_H(:,:,i), 'mask', [1 1 0 1 0 1]);
    
    % NOTA: N�o est� a funcionar para a junta 3...
end



%% MENU ("main")

% Variaveis MENU
select = 0;
STOP = 9;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'a) Matriz PJ_DH e O T G',...
                                                   'b) c) q = [ 0�  0�  0�]',...
                                                   'b) c) q = [10� 20� 30�]',...
                                                   'b) c) q = [90� 90� 90�]',...
                                                   'Robot q = [ 0�  0�  0�]',...
                                                   'Robot q = [10� 20� 30�]',...
                                                   'Robot q = [90� 90� 90�]',...
                                                   'Cinem�tica Inversa',...
                                                   'Quit');  
                                                
    % a) Matriz dos parametros de Denavith-Hartenberg: PJ_DH
    if select == 1  
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Matriz dos par�metros de Denavith-Hartenberg: PJ_DH')
        disp('______________________________________________________________________')
        disp(' ')
        robot.display
        disp(' ')
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Cinem�tica Directa: O T G')
        disp('______________________________________________________________________')
        disp(' ')
        disp(T0_G)
        disp(' ')
        disp('______________________________________________________________________')
    disp('#######################################################################')   
    end  
    
    % b) Matrizes de Cinem�tica Directa de 0 A 2 e 0 A H
    if select == 2 || select == 3 || select == 4
        
        i = select - 1;
        
        disp(' ')
        disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) '� ' num2str(rad2deg(q(i,2))) '� ' num2str(rad2deg(q(i,3))) '� ]'])
        disp(' ')
        disp('A02:')
        disp(' ')
        disp(A0_2(:,:,i))
        disp(' ')
        disp('c) Confirma��o usando a toolbox Robotics:')
        disp(' ')
        disp(T02(:,:,i))
        disp('______________________________________________________________________') 
        disp(' ')
        disp('A0H:')
        disp(' ')
        disp(A0_H(:,:,i))
        disp(' ')
        disp('c) Confirma��o usando a toolbox Robotics:')
        disp(' ')
        disp(double(T0H(:,:,i)))
        disp(' ')
        disp('______________________________________________________________________')   
    end

    % Representa��o gr�fica dos Rob�s
    if select == 5 || select == 6 || select == 7
        
        i = select - 4;
        
        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de topo do Robot -------------------------------------
        robot.plot([q(i,:) 0], 'workspace', [-15 15 -2 10 -2 2],...
                   'reach', 1,...
                   'scale', 1,...
                   'zoom', 0.25,...
                   'view',...
                   'top');
         
    end
    
    % Cinem�tica Inversa
    if select == 8
        disp('d) e) Solu��o de Cinem�tica Inversa ')
        disp(' ')

        disp('Matriz simb�lica do Mundo ao Gripper: O T G')
        disp(' ')
        disp(T0_G)
        disp(' ')
        disp('Matriz simb�lica do Mundo ao Bra�o: O T 2')
        disp(' ')
        disp(T0_2)
        disp(' ')
        disp('______________________________________________________________________')   

        % Conjuntos das juntas para cada caso da al�nea anterior:
        disp(' ')
        disp('Conjunto de solu��es: ')
        
        for i=1:3
            disp(['b) ' num2str(i) ')'])
            disp(' ')
            disp('Caso positivo')
            disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) '� ' num2str(rad2deg(q(i,2))) '� ' num2str(rad2deg(q(i,3))) '� ]'])
            disp(' ')

            disp('Caso negativo:')
            disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q_(i,1))) '� ' num2str(rad2deg(q_(i,2))) '� ' num2str(rad2deg(q_(i,3))) '� ]'])
            disp(' ')
            disp(' ')
            if i<3
            disp('c) Confirma��o usando a toolbox Robotics:')
            disp(' ')
            disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q_ikine(i,1))) '� ' num2str(rad2deg(q_ikine(i,2))) '� ' num2str(rad2deg(q_ikine(i,3))) '� ]'])
            disp(' ')
            end
            
            disp('______________________________________________________________________')   
        end
    end
    
    % clear workspace
    if select == STOP
       close all; 
    end
    
end













