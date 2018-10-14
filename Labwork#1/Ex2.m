close all
clear all
clc

disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp('%%    [Rob�tica - 27/09/2018 ~ 14/10/2018] LABWORK#1 - PROBLEMA 1    %%')
disp('%%                                                                   %%')
disp('%%                   Frederico Vaz, n� 2011283029                    %%')
disp('%%                   Paulo Almeida, n� 2010128473                    %%')
disp('%%                                                                   %%')
disp('%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
disp(' ')


%% Descri��o: 
% Considere o ambiente de trabalho que se apresenta, onde o objectivo � usar a
% garra de uma manipulador colocada em M para executar a seguinte sequ�ncia
% de ac��es:
% 1. Pegar no objecto A;
% 2. Inseri-lo no objecto B;
% 3. Encaixar o conjunto no objecto C;     
% [ambiente apresentado em imagem no enunciado]
% De forma a Executar a sequ�ncia: 

%% Desenhar objectos A, B, C, M

global pointsA pointsB pointsC pointsM

% points objecto A - Matriz Nx3 - [x, y, z]
pointsA = [ 0 0 0; 
            0 0 1;
            0 1 1;
            0 1 2;
            0 2 2; 
            0 2 1;
            0 3 1;
            0 3 0 ];
        
% points objecto B
pointsB = [ 0 0 0;
            0 0 2;
            1 0 2;
            1 0 1;
            2 0 1;
            2 0 2;
            3 0 2;
            3 0 0 ];
        
% points objecto C
pointsC = [ 0 0 0;
            1 0 4;
            1 0 1;
            4 0 1;
            4 0 4;
            5 0 0 ];
        
% points objecto M
pointsM = [ 2 1 0;
            2 0 0; 
            1 1 0;
            0.75 1 0;
            0.75 2 0;
            3 2 0;
            3 4 0;
            4 4 0;
            4 2 0;
            6.25 2 0;
            6.25 1 0;
            6 1 0;
            5 0 0;
            5 1 0 ];
  
% Cria pontos 3D baseado nos pontos 2D de uma face      
pointsA = createObject(pointsA, [3 0 0]);
pointsB = createObject(pointsB, [0 3 0]); 
pointsC = createObject(pointsC, [0 3 0]); 
pointsM = createObject(pointsM, [0 0 1]); 
        
% Desenha objectos projectados
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,2,1)
drawObject(pointsA, 'y');    
subplot(2,2,2)
drawObject(pointsB, 'g') 
subplot(2,2,3)
drawObject(pointsC, 'b') 
subplot(2,2,4)
drawObject(pointsM, 'r') 

% Calibra��o do referencial dos objectos 
% Pretende-se colocar a origem do referencial num ponto do objecto onde
% permita a maior manipula��o dos objectos com o gripper

% Refer�ncia Faces paralelas a YZ

% Objecto A: -3/2 unidades em Y
rTa = trans(0, 0, 0, [0 -1.5 0]');
% Objecto B: Roda 90� em Z & -3/2 unidades em Y
rTb = trans(90, 0, 0, [0 -1.5 0]');
% Objecto C: Roda 90� em Z & 3 em X -5/2 em Y, unidades
rTc = trans(90, 0, 0, [3 -5/2 0]');
% Objecto M: Roda 90� &  em X e em Y, unidades
rTm = trans(90, 0, 0, [1 -7/2 0]');

% Nota: Ajustou-se o Referencial do Gripper e A 
figure('units','normalized','outerposition',[0 0 1 1]);
subplot(2,2,1)
pointsA = transPoints(rTa, pointsA);
drawObject(pointsA, 'y')    
subplot(2,2,2)
pointsB = transPoints(rTb, pointsB);
drawObject(pointsB, 'g') 
subplot(2,2,3)
pointsC = transPoints(rTc, pointsC);
drawObject(pointsC, 'b') 
subplot(2,2,4)
pointsM = transPoints(rTm, pointsM);
drawObject(pointsM, 'r') 


%% Configura��o dos objectos no ambiente de trabalho ou mundo

% Vectores de transla��o - Posi��es Iniciais
poseA = [  0 10 0 ]; % -3 10 
poseB = [ 15 20 0 ];
poseC = [ 15  0 0 ];
poseM = [  5 20 2 ];

% Transla��es Puras (dos referenciais)

% Matriz de transforma��o que relaciona A em R 
rTa = trans(180, 0, 0, poseA');
% Matriz de transforma��o que relaciona B em R
rTb = trans(-90, 0, 0, poseB');
% Matriz de transforma��o que relaciona C em R
rTc = trans(-90, 0, 0, poseC');
% Matriz de transforma��o que relaciona M em R
rTm = trans(-90, 0, 0, poseM');

% Transforma points do objecto em rela��o ao referencial R ou mundo
figure('units','normalized','outerposition',[0 0 1 1]);
animateObject(rTm, rTa, rTb, rTc); 

%% Rela��es entre objectos seguindo a sequ�ncia M -> A -> B -> C

mTa = trans_inverse(rTm) * rTa;
aTb = trans_inverse(rTa) * rTb;
bTc = trans_inverse(rTb) * rTb;

RTC = rTm * mTa * aTb * bTc;

% Este seria usado para o ponto de perpectiva do Gripper

%% 1) 
disp('In�cio da Anima��o')
disp('1) Pegar no objecto A com o Gripper (objecto M)')
global delay

% Delay da anima��o da sequ�ncia do ponto de vista do Referencial
delay = 0.0001;

% Transforma��es sobre o Gripper M

% Transla��o de 10 unidade em Y
for i=0:1:10
    rTm_1 = rTm * trans(0, 0, 0, [i 0 0]');
    animateObject(rTm_1, rTa, rTb, rTc);  
end

% Rota��o de -90� em Z
for i=0:10:90
    rTm_2 = rTm_1 * trans(-i, 0, 0, [0 0 0]');
    animateObject(rTm_2, rTa, rTb, rTc);  
end

% Transla��o de -2 unidade em Z
for i=0:0.5:2
    rTm_3 = rTm_2 * trans(0, 0, 0, [0 0 -i]');
    animateObject(rTm_3, rTa, rTb, rTc);  
end

% Transla��o de 5 unidade em X
for i=0:0.5:5
    rTm_4 = rTm_3 * trans(0, 0, 0, [i 0 0]'); 
    animateObject(rTm_4, rTa, rTb, rTc);  
end

%------------------------------------------------------------------------------------------
disp('2) Inseri-lo no objecto B')
% As origens dos referenciais do gripper com objectos est�o coicidentes
% n�o sendo necess�rio para a anima��o o c�culo de todos, mas faremos 
% a actualiza��o de todos - A, B e C

% Transforma��es sobre o objecto A e Gripper M

% Transla��o de 5 unidade em Z
for i=0:1:5
    rTm_5 = rTm_4 * trans(0, 0, 0, [0 0 i]');
    rTa_1 = rTa * trans(0, 0, 0, [0 0 i]');
    animateObject(rTm_5, rTa_1, rTb, rTc);  
end

% Transla��o de 15 unidade em X
for i=0:1:15
    rTm_6 = rTm_5 * trans(0, 0, 0, [-i 0 0]'); 
    rTa_2 = rTa_1 * trans(0, 0, 0, [-i 0 0]');
    animateObject(rTm_6, rTa_2, rTb, rTc);  
end

% Rota��o de -90� em Z
for i=0:10:90
    rTm_7 = rTm_6 * trans(-i, 0, 0, [0 0 0]');
    rTa_3 = rTa_2 * trans(-i, 0, 0, [0 0 0]');
    animateObject(rTm_7, rTa_3, rTb, rTc); 
end

% Transla��o de 10 unidade em Y
for i=0:1:10
    rTm_8 = rTm_7 * trans(0, 0, 0, [i 0 0]');
    rTa_4 = rTa_3 * trans(0, 0, 0, [i 0 0]');
    animateObject(rTm_8, rTa_4, rTb, rTc); 
end

% Rota��o de 180� unidade em X
for i=0:10:180
    rTm_9 = rTm_8 * trans(0, 0, i, [0 0 0]');
    rTa_5 = rTa_4 * trans(0, 0, i, [0 0 0]');
    animateObject(rTm_9, rTa_5, rTb, rTc); 
end

% Transla��o de 2 unidade em Z
for i=0:1:2
    rTm_10 = rTm_9 * trans(0, 0, 0, [0 0 i]');
    rTa_6 = rTa_5 * trans(0, 0, 0, [0 0 i]');
    animateObject(rTm_10, rTa_6, rTb, rTc);  
end

% Transla��o de 2 unidade em Z do Gripper
for i=0:1:1
    rTm_11 = rTm_10 * trans(0, 0, 0, [0 0 i]');
    animateObject(rTm_11, rTa_6, rTb, rTc);  
end

%------------------------------------------------------------------------------------------
disp('3) Encaixar o conjunto no objecto C')

% Transla��o de -1 unidade em Z
for i=0:0.5:1
    rTm_12 = rTm_11 * trans(0, 0, 0, [0 0 -i]');
    rTa_7 = rTa_6 * trans(0, 0, 0, [0 0 -i]');
    rTb_1 = rTb * trans(0, 0, 0, [0 0 i]');
    animateObject(rTm_12, rTa_7, rTb_1, rTc);  
end

% Rota��o de 180� em Z
for i=0:10:180
    rTm_13 = rTm_12 * trans(i, 0, 0, [0 0 0]');
    rTa_8 = rTa_7 * trans(i, 0, 0, [0 0 0]');
    rTb_2 = rTb_1 * trans(-i, 0, 0, [0 0 0]');
    animateObject(rTm_13, rTa_8, rTb_2, rTc);  
end

% Transla��o de 19 unidade em X
for i=0:1:19
    rTm_14 = rTm_13 * trans(0, 0, 0, [i 0 0]');
    rTa_9 = rTa_8 * trans(0, 0, 0, [i 0 0]');
    rTb_3 = rTb_2 * trans(0, 0, 0, [-i 0 0]');
    animateObject(rTm_14, rTa_9, rTb_3, rTc);  
end

%------------------------------------------------------------------------------------------
disp(' ')
disp('NOTA: A nossa fun��o drawObject utiliza um algoritmo nosso, que serve')
disp('para construir qualquer paralepipedo de duas faces paralelas, sendo que')
disp('que uma das faces � criada mpondo um offset.')
disp('')
disp('� criado o objecto em 3D e guardados os pontos em 3D. Assim � possivel')
disp('aplicar transforma��es a todos esses pontos e ver a anim��o em 3D')
%------------------------------------------------------------------------------------------






























