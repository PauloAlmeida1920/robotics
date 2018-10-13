
%% Cálcula os ângulos de Euler Alpha, Beta, Gama 
% Tendo como parametro de entrada a Matriz de Rotação 
% Slides III p.20

function [ alpha, beta, gama ] = euler_inv(R)
    
    beta = atan2( -R(3,1), sqrt( R(1,1)^2 + R(2,1)^2 ) );
    
    if single( rad2deg(beta) ) == 90 
        
        alpha = 0;
        gama = atan2( R(1,2), R(2,2) );
        
    else if single( rad2deg(beta) ) == -90
        
         alpha = 0;
         gama = -atan2( R(1,2), R(2,2) );
         
    else 
        
        alpha = atan2( R(2,1)/cos(beta), R(1,1)/cos(beta) );
        gama = atan2( R(3,2)/cos(beta), R(3,3)/cos(beta) );
        
    end
    
end