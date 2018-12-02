clear all
clc

%% Cinemática Inversa

syms nx ny nz sx sy sz ax ay az tx ty tz

T0_G_nsat = [ nx sx ax tx;
              ny sy ay ty;
              nz sz az tz;
               0  0  0  1 ];

% Cinemática Inversa das juntas: theta1 e d2

t0_G = T0_G(1:3,4);
Rz0_G = T0_G(1:3,3);

% 0 t 2 = 0 t G - L4 * 0 Rz G
t0_2 = simplify( t0_G - L4 * Rz0_G )

% tx' e ty'
tx_l = t0_2(1);
ty_l = t0_2(2);

% Vector Simbólico
t0_G = T0_G_nsat(1:3,4);
Rz0_G = T0_G_nsat(1:3,3);
t0_2_syms = simplify( t0_G - L4 * Rz0_G )

                     
% Cinemática Inversa das juntas: theta3

% Auxiliar Base ao Braço -> 2 T 0:
T0_1 = Ti(:,:,1);
T1_2 = Ti(:,:,2);

T0_2 = simplify( T0_1 * T1_2 );

T2_0 = simplify( inv(T0_2) );

% Matriz Simbólica 2 T G: De forma a conhecer que valores usar dados em O T G
T2_G_nsat = simplify( T2_0 * T0_G_nsat )

% Resultado - Gripper no Elo 2 -> 2 T G:
T2_G = simplify(  T2_0 * T0_G )              
              


%% Teste: 

% Posição HOME:
Tb_f = [ -cos(alfa) 0  sin(alfa)  40;
          sin(alfa) 0  cos(alfa)  20;
                  0 1          0   0;
                  0 0          0   1  ];
     
alf = pi/4;
              
Tb_f = eval(subs(Tb_f, alfa, alf))  

[ q1 ] = inverse_kinematics_ex2(Tb_f, alf) 


% Confirmação: 

% Juntas em symbolic p/ resolver o Jacobiano
q_aux = [ theta1 d2 theta3 ];

Tb_f = eval(subs(T0_G, [q_aux L4], [q1 10]))   
              
              