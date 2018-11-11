clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robótica - 23/10/2018 ~ 11/11/2018] LABWORK#2 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')

%% Robô 3-DOF: L1 = 4; L2 = 3 e L3 = 2

syms theta1 theta2 theta3

% Offset/comprimentos dos elos (fixos)
syms L1 L2 L3

% Junta Rotacional ou Prismática
R = 1; P = 0;

% a) Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH
%____________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%____________________________________________________________________________________      
PJ_DH = [  theta1      0     L1        0         0           R;   % Junta Rotacional
%____________________________________________________________________________________
           theta2      0     L2        0         0           R;   % Junta Rotacional
%____________________________________________________________________________________
           theta3      0      0     pi/2      pi/2           R;   % Junta Rotacional
%____________________________________________________________________________________
                0     L3      0        0      pi/2           R ]; % Gripper Fixo
%____________________________________________________________________________________
            
% A cinemática directa até ao Gripper
[ T0_G, Ti ] = MGD_DH(PJ_DH);

% Offset/comprimentos dos elos (fixos)
PJ_DH =  eval(subs(PJ_DH, [L1 L2 L3], [4 3 2]));

% Criar Links Juntas Rotacionais -> o theta variável
for i=1:4
    L(i) = Link('d',eval(PJ_DH(i,2)), 'a', eval(PJ_DH(i,3)), ...
           'alpha', eval(PJ_DH(i,4)), 'offset', eval(PJ_DH(i,5)));
end

robot = SerialLink(L, 'name', 'Robô Planar 3-DOF RRR');
robot_T0_2 = SerialLink(L(1:2));


%% b) Matrizes de Cinemática Directa de 0 A 2 e 0 A H;  c) Confirmação

% 3 casos de valores nas juntas para o Robô Planar 3-DOF - Gripper fixo
q = [ deg2rad(0)  deg2rad(0)  deg2rad(0); 
      deg2rad(10) deg2rad(20) deg2rad(30);
      deg2rad(90) deg2rad(90) deg2rad(90) ];
  
% i) ii) iii)
for i=1:3
    
    T0_1 = eval(subs(Ti(:,:,1), [L1 theta1], [4 q(i,1)])); 
    T1_2 = eval(subs(Ti(:,:,2), [L2 theta2], [3 q(i,2)])); 
    T2_I = eval(subs(Ti(:,:,3), theta3, q(i,3))); 
    TI_H = eval(subs(Ti(:,:,4), L3, 2));    
   
    A0_2(:,:,i) = T0_1 * T1_2;
    A0_H(:,:,i) = T0_1 * T1_2 * T2_I * TI_H;
    
    
    % c) Confirmação das Matrizes usando a robotics toolbox 
    T02(:,:,i) = robot_T0_2.fkine(q(i,1:2));
    T0H(:,:,i) = robot.fkine([q(i,:) 0]);
    
end


%% d) e) Solução da Cinemática Inversa 

% Matriz simbólica do Mundo ao Braço: O T 2
T0_1 = Ti(:,:,1);
T1_2 = Ti(:,:,2);

T0_2 = simplify( T0_1 * T1_2 );


% P/ os conjuntos das juntas da alínea anterior:
for i=1:3
    
    [q(i,:), q_(i,:)] = inverse_kinematics_ex2(A0_H(:,:,i));
    
end

% Confirmação usando a robotics toolbox
for i=1:2

    q_ikine(i,:) = robot.ikine(A0_H(:,:,i), 'mask', [1 1 0 1 0 1]); % [x y z roll pitch yaw] 
    
end



%% MENU ("main")

% Variaveis MENU
select = 0;
STOP = 9;

while(select ~= STOP)
    
    select = menu('Seleccione a acao a realizar:', 'a) Matriz PJ_DH e O T G',...
                                                   'b) c) q = [ 0º  0º  0º]',...
                                                   'b) c) q = [10º 20º 30º]',...
                                                   'b) c) q = [90º 90º 90º]',...
                                                   'Robot q = [ 0º  0º  0º]',...
                                                   'Robot q = [10º 20º 30º]',...
                                                   'Robot q = [90º 90º 90º]',...
                                                   'Cinemática Inversa',...
                                                   'Quit');  
                                                
    % a) Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH
    if select == 1  
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
    end  
    
    % b) Matrizes de Cinemática Directa de 0 A 2 e 0 A H
    if select == 2 || select == 3 || select == 4
        
        i = select - 1;
                
        disp('______________________________________________________________________')
        disp(' ')
        disp(['b) q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) 'º ' num2str(rad2deg(q(i,2))) 'º ' num2str(rad2deg(q(i,3))) 'º ]'])
        disp('______________________________________________________________________')
        disp(' ')
        disp('A02:')
        disp(' ')
        disp(A0_2(:,:,i))
        disp('c) Confirmação usando a toolbox Robotics:')
        disp(' ')
        disp(double(T02(:,:,i)))
        disp('______________________________________________________________________') 
        disp(' ')
        disp('A0H:')
        disp(' ')
        disp(A0_H(:,:,i))
        disp('c) Confirmação usando a toolbox Robotics:')
        disp(' ')
        disp(double(T0H(:,:,i)))
        disp(' ')
        disp('______________________________________________________________________')   
    end

    % Representação gráfica dos Robôs
    if select == 5 || select == 6 || select == 7
        
        i = select - 4;
        
        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de topo do Robot -------------------------------------
        robot.plot([q(i,:) 0], 'workspace', [-15 15 -2 10 -2 2],...
                   'reach', 1,...
                   'scale', 1,...
                   'zoom', 0.25,...
                   'jaxes', ...
                   'view',...
                   'top');
         
    end
    
    % Cinemática Inversa
    if select == 8
        close all;
        disp('______________________________________________________________________')
        disp(' ')
        disp('d) e) Solução de Cinemática Inversa ')
        disp('______________________________________________________________________')
        disp(' ')
        disp('Gripper no Mundo: O T G')
        disp(' ')
        disp(T0_G)
        disp('Mundo ao Braço: O T 2')
        disp(' ')
        disp(T0_2)
        disp(' ')
        disp('______________________________________________________________________')   

        % Conjuntos das juntas para cada caso da alínea anterior:
        disp(' ')
        disp('Conjunto de soluções: ')
        
        for i=1:3
            disp('______________________________________________________________________')
            disp(' ')
            disp(['b) q' num2str(i)])
            disp('______________________________________________________________________')
            disp(' ')
            %disp('Caso positivo')
            disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) 'º ' num2str(rad2deg(q(i,2))) 'º ' num2str(rad2deg(q(i,3))) 'º ]'])
            disp(' ')

%             disp('Caso negativo:')
%             disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q_(i,1))) 'º ' num2str(rad2deg(q_(i,2))) 'º ' num2str(rad2deg(q_(i,3))) 'º ]'])
%             disp(' ')
%             disp(' ')
            if i<3
            disp('c) Confirmação usando a toolbox Robotics:')
            disp(' ')
            disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q_ikine(i,1))) 'º ' num2str(rad2deg(q_ikine(i,2))) 'º ' num2str(rad2deg(q_ikine(i,3))) 'º ]'])
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













