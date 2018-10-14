
%% Vector de rota��o r e respectivo �ngulo de rota��o phi a partir da Matriz de Rota��o

function [ r, phi ] = vectorRot_inv(R)

    % �ngulo de rota��o 
    phi = acos( ( R(1,1) + R(2,2) + R(3,3) - 1 ) / 2 );
    
    % Vector de rota��o
    r = 1 / ( 2 * sin(phi) ) * [ R(3,2)-R(2,3) R(1,3)-R(3,1) R(2,1)-R(1,2) ]';

end