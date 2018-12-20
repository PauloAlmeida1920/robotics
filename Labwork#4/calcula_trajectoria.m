
%% DESCRIÇÃO: Implementa a equação polinomial de 3ª ordem com componentes de posição e velocidade;
% garante que a trajectória satisfaz uma posição e velocidade final
% desejada.
% - Continuidade e suavidade nas velocidades e acelerações das juntas;
% - Evitar solicitações desmesuradas e irregulares nos actuadores.
%###################################################################################################
% ARGUMENTOS: 
%       - <t> : variável simbólica para o tempo;
%
%       - <ti> : instante inicial (t = ti);
%
%       - <qi> : valor das juntas na posição inicial (t = ti);
%
%       - <qf> : valor das juntas na posição final;
%
%       - <delta_t> : diferença entre tempo no instante final e tempo no
%       instante inicial;
%
%       - <v_qi> : velocidade das juntas na posição inicial (com t = ti)
%
%       - <v_qf> : velocidade dsa juntas na posição final
%
%###################################################################################################

function [ pos, q_traj ] = calcula_trajectoria(oTg, t, q_aux, q, v_q, h)

syms theta1 d2 theta3

count = 1;

for i=1:size(t,2)-1 % Para cada instante i - linhas (um ponto)
    for th=t(i):h:t(i+1)-h % Amostragem de pontos da trajectoria entre pontos/instantes
        for k=1:1:size(q,2) % Para cada junta k - colunas

            ti = t(i);
            tf = t(i+1);
            qi = q(i,k);
            qf = q(i+1,k);
            v_qi = v_q(i,k);
            v_qf = v_q(i+1,k);
            
            delta_t = tf - ti;
            
            q_traj(count,k) = qi +                                    ...  % a0
                                                                      ...
                              v_qi*(th - ti) +                        ...  % a1*t
                                                                      ...
                              ((3/delta_t^2)*(qf-qi)-(2/delta_t)*v_qi ...
                              -(1/delta_t)*v_qf)*(th-ti)^2            ...  % a2*t^2
                                                                      ...
                              -((2/delta_t^3)*(qf-qi)-(1/delta_t^2)*  ...
                              (v_qf+v_qi))*(th-ti)^3;                      % a3*t^3
            
        end
        % Calcula posições
        oTg_ = eval(subs(oTg, q_aux, q_traj(count,:)));
        pos(count,:) = [ oTg_(1,4) oTg_(2,4) oTg_(3,4) ];
        
        count = count + 1;
    end
end
end







