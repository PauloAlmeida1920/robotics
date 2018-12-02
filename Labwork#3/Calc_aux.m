clear all
clc

%% Cálculos auxiliares e Testes:

% Posição HOME:
Tb_f = [ -cos(alfa) 0  sin(alfa)  40;
          sin(alfa) 0  cos(alfa)  20;
                  0 1          0   0;
                  0 0          0   1  ];

% Tb_f = [ 0 -cos(alfa) sin(alfa)  40;
%          0  sin(alfa) cos(alfa)  20;
%          1          0         0   0;
%          0          0         0   1  ];
  

% Tb_f = [ 1 0  0  40;
%          0 1  0  20;
%          0 0  1   0;
%          0 0  0   1  ];

     
alf = pi/4;
              
Tb_f = eval(subs(Tb_f, alfa, alf))  


[ q1 ] = inverse_kinematics_ex2(Tb_f, alf) 
[ q2 ] = inverse_kinematics_ex2_b(Tb_f, alf) 


%%
              
% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 d2 theta3 ];


Tb_f = eval(subs(T0_G, [q_aux L4], [pi/4 36.0555 pi/4 10]))              
              
% Confirmação usando a robotics toolbox
%q_bytoolbox = robot.ikine(Tb_f, 'mask', [1 1 0 1 0 0]) % [x y z roll pitch yaw]              
              
  

%% Cinemática Inversa das juntas do Punho Esférico: theta3 theta4 theta 5

syms nx ny nz sx sy sz ax ay az tx ty tz

T0_G_nsat = [ nx sx ax tx;
              ny sy ay ty;
              nz sz az tz;
               0  0  0  1 ];
           
           
% Auxiliar Base ao Braço -> 2 T 0:
T0_1 = Ti(:,:,1);
T1_2 = Ti(:,:,2);

T0_2 = simplify( T0_1 * T1_2 );

T2_0 = simplify( inv(T0_2) );

% Matriz Simbólica 2 T G: De forma a conhecer que valores usar dados em O T G
T2_G_nsat = simplify( T2_0 * T0_G_nsat )

% Resultado - Gripper no Elo 2 -> 2 T G:
T2_G = simplify(  T2_0 * T0_G )              
              
              
              
              