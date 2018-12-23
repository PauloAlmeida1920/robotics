
%% Cáculo da cinemática directa segundo o algortimo de Denavith-Hartenberg 
%  é obtida através da mutiplicação das matrizes/transformações 
%  correspondentes a cada elo/junta de um robô

function [ T0_G, Ti ] = MGD_DH(PJ_DH)

    T0_G = eye(4);

    % Ciclo para percorrer as juntas todas do robot e gerar as
    % respectivas matrizes de transformação;
    for i=1:size(PJ_DH,1)

        if PJ_DH(i,6) == 1        % Se a junta é de Rotação

            thetai = PJ_DH(i,1)+PJ_DH(i,5);
            di = PJ_DH(i,2);
        elseif PJ_DH(i,6) == 0    % Se a junta é Prismática

            thetai = PJ_DH(i,1);
            di = PJ_DH(i,2)+PJ_DH(i,5);
        end

        ai = PJ_DH(i,3);
        alfai = PJ_DH(i,4);

        % Parâmetros de entrada: thetai di  alfai  ai
        Ti(:,:,i) = jointMatrix(thetai, di, alfai, ai);
        T0_G = T0_G * Ti(:,:,i);

    end
    
    
    T0_G = simplify(T0_G);
    Ti = simplify(Ti);
    
end