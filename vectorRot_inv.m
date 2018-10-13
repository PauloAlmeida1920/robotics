
%% Vector de rotação r e respectivo ângulo de rotação phi a partir da Matriz de Rotação

function [ r, phi ] = vectorRot_inv(R)

    % Ângulo de rotação 
    phi = acos( ( R(1,1) + R(2,2) + R(3,3) - 1 ) / 2 );
    
    % Vector de rotação
    r = 1 / ( 2 * sin(phi) ) * [ R(3,2)-R(2,3) R(1,3)-R(3,1) R(2,1)-R(1,2) ]';

end