
%% Matriz de transformação do elo/junta i de um robô

function T = jointMatrix(theta, d, alfa, a)  

    T = [   cos(theta)     -sin(theta)*cos(alfa)      sin(theta)*sin(alfa)     a*cos(theta);
            sin(theta)      cos(theta)*cos(alfa)     -cos(theta)*sin(alfa)     a*sin(theta);
            0               sin(alfa)                 cos(alfa)                d;
            0               0                           0                      1                ];
        
end