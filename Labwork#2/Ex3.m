clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Rob�tica - 23/10/2018 ~ 11/11/2018] LABWORK#2 - PROBLEMA 3    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, n� 2011283029                    %%')
disp('%%                   Paulo Almeida, n� 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


%% Rob� 3-DOF: L1 = 4; L2 = 3 e L3 = 2

syms theta1 theta2 theta3 phi l1 l2 l3 l d

% 1) Matriz dos par�metros de Denavith-Hartenberg: PJ_DH
disp('a) Matriz dos par�metros de Denavith-Hartenberg: PJ_DH')
disp(' ')

% Junta Rotacional ou Prism�tica
R = 1; P = 0;


% Rob�1
%           thetai   di    ai   alfai  offseti  jointtypei       
PJ_DH1 = [  theta1    0    l1       0        0           R;   % Junta Rotacional
            theta2    0    l2       0        0           R;   % Junta Rotacional
            theta3    0     0    pi/2     pi/2           R;   % Junta Rotacional
                 0   l3     0       0     pi/2           P ]; % Gripper Fixo: theta fixo

% Rob�2            
%           thetai   di    ai   alfai  offseti  jointtypei       
PJ_DH2 = [     phi    0     l    pi/2     pi/2           R;   % Junta Rotacional
                 0    d     0       0        0           P;   % Junta Prism�tica
                 0    0     0       0     pi/2           R ]; % Gripper Fixo: theta fixo


[ oTg_1, Ti_1 ] = MGD_DH(PJ_DH1);       % A cinem�tica directa at� o Gripper 
[ oTg_2, Ti_2 ] = MGD_DH(PJ_DH2);       % A cinem�tica directa at� o Gripper 

oTg_1 = simplify(oTg_1);
Ti_1 = simplify(Ti_1);

oTg_2 = simplify(oTg_2);
Ti_2 = simplify(Ti_2);

disp(' ')
disp('Cinem�tica Directa c/ vari�veis simb�licas:')
disp(' ')
disp('Rob� 1')
disp(oTg_1)
disp(' ')
disp('Rob� 2')
disp(oTg_2)

disp('#######################################################################')

%% 4) Representa��o gr�fica
disp('c) Representa��o gr�fica')
disp(' ')

% Rob�1
% Cumprimentos dos elos
L1 = 4; L2 = 3; L3 = 2;
% Junta Rotacional ou Prism�tica
R = 1; P = 0;


%           thetai   di    ai   alfai  offseti  jointtypei       
PJ_DH1 = [  theta1    0    L1       0        0           R;   % Junta Rotacional
            theta2    0    L2       0        0           R;   % Junta Rotacional
            theta3    0     0    pi/2     pi/2           R;   % Junta Rotacional
                 0   L3     0       0     pi/2           R ]; % Gripper Fixo: theta fixo

% Rob�2  
L = 2;
%           thetai   di    ai   alfai  offseti  jointtypei       
PJ_DH2 = [     phi    0     L    pi/2     pi/2           R;   % Junta Rotacional
                 0    d     0       0        0           P;   % Junta Prism�tica
                 0    0     0       0     pi/2           R ]; % Gripper Fixo: theta fixo


%% INICIALIZA��O DO ROBOT: CRIAR LINKS

for i = 1 : size(PJ_DH1,1)
    
    if PJ_DH1(i,6) == R              % Juntas Rotacionais
        
        L(i) = Link('d',eval(PJ_DH1(i,2)),...
                    'a', eval(PJ_DH1(i,3)),...
                    'alpha', eval(PJ_DH1(i,4)),...
                    'offset', eval(PJ_DH1(i,5)));
    end
    
    if PJ_DH1(i,6) == P              % Junta Prismática
        
        L(i) = Link('theta',eval(PJ_DH1(i,1)),...
                    'a', eval(PJ_DH1(i,3)),...
                    'alpha', eval(PJ_DH1(i,4)),...
                    'offset', eval(PJ_DH1(i,5)));
%                     'qlim', [0 1]);
    end

end
% Criar Links Rob�1
% for i=1:4
%     L(i) = Link('d',eval(PJ_DH1(i,2)), 'a', eval(PJ_DH1(i,3)), ...
%            'alpha', eval(PJ_DH1(i,4)), 'offset', eval(PJ_DH1(i,5)));
% end

% % Criar Links Rob�2
% L(5) = Link('d',eval(PJ_DH2(i,2)), 'a', eval(PJ_DH2(i,3)), ...
%            'alpha', eval(PJ_DH2(i,4)), 'offset', eval(PJ_DH2(i,5)));

% L(6) = Link('theta',eval(PJ_DH2(i,2)), 'a', eval(PJ_DH2(i,3)), ...
%            'alpha', eval(PJ_DH2(i,4)), 'offset', eval(PJ_DH2(i,5)));

% L(7) = Link('d',eval(PJ_DH2(i,2)), 'a', eval(PJ_DH2(i,3)), ...
%            'alpha', eval(PJ_DH2(i,4)), 'offset', eval(PJ_DH2(i,5)));      


% Usando a fun��es da toolbox Robotics
disp('Tabela Denavith-Hartenberg:')

robot = SerialLink(L, 'name', 'Rob� 1 Rob� 2')

%%
%       theta1  theta2 theta3  phi  d
q = [    0       0      0     0     0 ];
 
  
for i=1:1
    disp('--------------------------------------------------------------------')
    disp(' ')
    disp(['q' num2str(i) ' = [ ' num2str(rad2deg(q(i,1))) '� ' num2str(rad2deg(q(i,2))) '� ' num2str(rad2deg(q(i,3))) '� ]'])
    disp(' ')
    
    [ q(i,:) q_n(i,:)] = inverse_kinematics_ex2();
    
    disp(' ')
    disp(['Visualizar Rob�s c/ q' num2str(i) '-> Enter'])
    pause;
    robot.plot(q(i,:), 'workspace', [-10 10 -2 5 -2 2], 'reach',1);
end
disp(' ')
disp('########################################################################')
