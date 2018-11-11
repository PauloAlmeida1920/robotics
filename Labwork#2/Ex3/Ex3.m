clear all
close all
clc

format short 

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Robótica - 23/10/2018 ~ 11/11/2018] LABWORK#2 - PROBLEMA 3    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, nº 2011283029                    %%')
disp('%%                   Paulo Almeida, nº 2010128473                    %%')
disp('%%                                                                   %%')
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


%% Problema 3

disp('****************************** Ex3 ************************************')

syms theta1 theta2 theta3 phi

% Offset/comprimentos dos elos (fixos)
syms L1 L2 L4 l d

% Junta Rotacional ou Prismatica:
R = 1; P = 0;


% Robot 1 [RRR] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
%____________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%____________________________________________________________________________________
PJ_DH1 = [  theta1      0     L1        0         0           R;    % Junta prismática
%____________________________________________________________________________________
            theta2      0     L2        0         0           R;    % Junta prismática
%____________________________________________________________________________________
            theta3      0      0     pi/2      pi/2           R;    % Punho esférico
%____________________________________________________________________________________
                 0     L4      0        0      pi/2           R ];    % Punho esférico
%____________________________________________________________________________________


% Robot 2 [R] - Matriz dos parametros de Denavith-Hartenberg: PJ_DH
%____________________________________________________________________________________
%          thetai  |  di  |  ai |  alfai | offseti | jointtypei
%____________________________________________________________________________________
PJ_DH2 = [    phi      0      l     pi/2         0           R;    % Junta prismática
%____________________________________________________________________________________
                0      d      0        0         0           R ];    % Junta prismática
%____________________________________________________________________________________


% A cinemática directa da base até ao Gripper: 
[ T0_G1, Ti_1 ] = MGD_DH(PJ_DH1);
[ T0_G2, Ti_2 ] = MGD_DH(PJ_DH2);

% Offset/comprimentos dos elos (fixos)
PJ_DH1 =  eval(subs(PJ_DH1, [L1 L2 L4], [sqrt(2)*2 sqrt(2)*2 sqrt(7)*2]));
PJ_DH2 =  eval(subs(PJ_DH2, [l d], [2 1]));


%% INICIALIZAÇÃO DO ROBOT: CRIAR LINKS

for i = 1 : size(PJ_DH1,1)
    
     if PJ_DH1(i,6) == R              % Juntas Rotacionais
        
        L_1(i) = Link('d',eval(PJ_DH1(i,2)),...
                    'a', eval(PJ_DH1(i,3)),...
                    'alpha', eval(PJ_DH1(i,4)),...
                    'offset', eval(PJ_DH1(i,5)));
     end
    
    if PJ_DH1(i,6) == P              % Junta Prismática
        
        L_1(i) = Link('theta',eval(PJ_DH1(i,1)),...
                    'a', eval(PJ_DH1(i,3)),...
                    'alpha', eval(PJ_DH1(i,4)),...
                    'offset', eval(PJ_DH1(i,5)),...
                    'qlim', [0 10]);
    end

end

for i = 1 : size(PJ_DH2,1)
    
     if PJ_DH2(i,6) == R              % Juntas Rotacionais
        
        L_2(i) = Link('d',eval(PJ_DH2(i,2)),...
                    'a', eval(PJ_DH2(i,3)),...
                    'alpha', eval(PJ_DH2(i,4)),...
                    'offset', eval(PJ_DH2(i,5)));
     end
    
    if PJ_DH2(i,6) == P              % Junta Prismática
        
        L_2(i) = Link('theta',eval(PJ_DH2(i,1)),...
                    'a', eval(PJ_DH2(i,3)),...
                    'alpha', eval(PJ_DH2(i,4)),...
                    'offset', eval(PJ_DH2(i,5)),...
                    'qlim', [0 10]);
    end

end

robot1 = SerialLink(L_1, 'name', 'Robô1');
robot2 = SerialLink(L_2, 'name', 'Robô2');


%% MENU ("main")

select = 0;
first = 0;
sair = 2;

while (select ~= sair)
    
    % Matriz dos parâmetros de Denavith-Hartenberg: PJ_DH e a O T G
    if first < 1
        disp('______________________________________________________________________')
        disp(' ')
        disp('a) Matrizes dos parâmetros de Denavith-Hartenberg: PJ_DH1 e PJ_DH2')
        disp('______________________________________________________________________')
        disp(' ')
        robot1.display
        robot2.display
        disp('______________________________________________________________________')
        
        first = first + 1;
    end
    
    select = menu('Seleccione a acção a realizar:', 'Animação dos Robôs',...
                                                    'Sair');
                                                
    % Animação dos Robôs
    if select == 1
        close all;

        % Desloca -> Roda -> Desloca

        d_ = -20:1:0;
        phi_ = zeros(1,size(d_,2));

        q(1,:) = inverse_kinematics_robot1_follow_robot2(phi_(1), d_(1));  

        figure('units','normalized','outerposition',[0 0 1 1]);
        % Prespectiva de lado do Robot
        robot1.plot([q(1,:) 0], 'workspace', [-6 6 -6 6 -1 1], 'reach', 1,...
                            'scale', 1, 'zoom', 0.5, 'jaxes');

        hold on

        robot2.plot([phi_(1) 0], 'workspace', [-6 6 -6 6 -1 1], 'reach', 1,...
                            'scale', 1, 'zoom', 0.5, 'jaxes');

        % Desloca                
        for i=2:size(d_, 2)-1

            q(i,:) = inverse_kinematics_robot1_follow_robot2(phi_(i), d_(i));

            robot1.animate([q(i,:) 0]);
            robot2.animate([phi_(end) 0]);

        end

        % Roda
        phi_ = 0:-0.05:-pi/2;
        for i=1:size(phi_, 2)

            q(i,:) = inverse_kinematics_robot1_follow_robot2(phi_(i), d_(end));

            robot1.animate([q(i,:) 0]);
            robot2.animate([phi_(i) 0]);

        end

        d_ = 0:-1:-20;
        % Desloca                
        for i=1:size(d_, 2)

            q(i,:) = inverse_kinematics_robot1_follow_robot2(phi_(end), d_(i));

            robot1.animate([q(i,:) 0]);
            robot2.animate([phi_(end) 0]);

        end

    end
    
    % clear workspace
    if select == sair
       close all; 
    end
    
end %fim do menu
