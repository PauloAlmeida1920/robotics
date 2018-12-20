
%% Cálcula velocidade nas juntas para T instantes

function [ v_q ] = calcula_velocidade(q, t)

% Caso particular: Instante inicial -> Velocidade igual a 0!
v_q_aux = zeros(1,size(q,2));

% 1º cálculamos as velocidas médias para determinar o seu sinal:
for i=1:1:size(t,2)-1 % Para cada instante i - linhas (um ponto)
    for k=1:1:size(q,2) % Para cada junta k - colunas
        
        v_q_aux(i+1,k) = ( q(i+1, k) - q(i, k) )/( t(i+1) - t(i) );
        
    end 
end
%
for i=1:1:size(t,2)-1 % Para cada instante i - linhas (um ponto)
    for k=1:1:size(q,2) % Para cada junta k - colunas
        
        % Sinal diferente, o ponto de passagem apresenta velocidade nula 
        if sign(v_q_aux(i,k)) ~= sign(v_q_aux(i+1,k))
            v_q(i,k) = 0;
        else
            v_q(i,k) = (1/2)*( v_q_aux(i,k) + v_q_aux(i+1,k));
        end  
        
    end 
end

% Caso particular: Instante final -> Velocidade igual a 0!
v_q = [ v_q; zeros(1,size(q,2)) ]; 
