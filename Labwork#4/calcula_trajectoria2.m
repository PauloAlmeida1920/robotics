
%% DESCRIÇÃO: Implementa a equação Parabola com componentes de posição e velocidade;
% garante que a trajectória satisfaz uma posição e velocidade final
% desejada.

function [ pos, q_traj ] = calcula_trajectoria2(oTg, a_max, q, t, h)

syms theta1 theta2

ti = t(1);
tim = t(2)-t(1);
tiM = t(3)-t(2); 
tf = t(3);

% Primeiro troço 
a(1,:) = sign(q(2,:)-q(1,:))*abs(a_max);
ta(1,:) = tim - sqrt( tim^2 - (2 .* ( q(2,:) - q(1,:) )) ./ a(1,:) );
v_q(1,:) = ( q(2,:) - q(1,:) ) ./ ( tim - (1/2) * ta(1,:) );

% Ultimo troço
a(2,:) = sign( q(2,:) - q(3,:) ) * abs( a_max );
ta(2,:) = tiM - sqrt( tiM^2 + (2 .* ( q(3,:) - q(2,:) )) ./ a(2,:) );
v_q(2,:) = ( q(3,:) - q(2,:) ) ./ ( tiM - (1/2) * ta(2,:) );

% Troço Intermédio
a(3,:) = sign( q(2,:) - q(1,:) ) * abs(a_max);
ta(3,:) = ( v_q(2,:) - v_q(1,:) ) ./ a(3,:);

% TA TB- TB+ TC
qi = q(1,:) + (1/2) * a(1,:) .* ta(1,:).^2;
qim = qi + v_q(1,:) .* ( tim - ta(1,:) - (1/2) .* ta(3,:));
qiM = qim + v_q(1,:) .* ta(3,:) + (1/2) * a(3,:) .* ta(3,:).^2;
qf = qiM + v_q(2,:) .* ( tiM - q(3,:) - (1/2) .* ta(3,:));

i=1;
for th=ti:h:tf-h
    for k=1:size(q,2) % Juntas
        
        if( th >= ti && th < ta(1,k) )
            
            q_traj(i,k) = q(1,k) + 0.5 * a(1,k) * (th-ti)^2;
            
        elseif( th >= ta(1,k) && th < tim - (1/2) * ta(3,k) )
            
            q_traj(i,k) = qi(k) + v_q(1,k) * (th - ta(1,k));
            
        elseif( th >= tim - (1/2) * ta(3,k) && th <  tim + 0.5 * ta(3,k))
            
            q_traj(i,k) = qim(k) + ...
                          v_q(1,k) * ( th - (tim - 0.5 * ta(3,k))) + ...
                          (1/2) * a(3,k) * ( th - (tim - (1/2) * ta(3,k) ))^2;
            
        elseif( th >= tim + (1/2) * ta(3,k) && th <  tf - ta(2,k) )
            
            q_traj(i,k) = qiM(k) + ...
                          v_q(2,k) * ( th - ( tim + (1/2) * ta(3,k) ));
            
        elseif( th >=  tf - ta(2,k) && th < tf )
            
            q_traj(i,k) = qf(k) + ...
                          v_q(2,k) * (th - ( tf - ta(2,k) )) + ...
                          (1/2) * a(2,k) * ( th - ( tf - ta(2,k) ))^2;
        end
    end
    % Calcula posições
    oTg_ = eval(subs(oTg, [theta1 theta2], q_traj(i,:)));
    pos(i,:) = [ oTg_(1,4) oTg_(2,4) oTg_(3,4) ];
    i = i + 1;
end
end


