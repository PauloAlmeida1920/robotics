
%% C�culo da cinem�tica directa segundo o algortimo de Denavith-Hartenberg 
%  � obtida atrav�s da mutiplica��o das matrizes/transforma��es 
%  correspondentes a cada elo/junta de um rob�

function [ oTg, Ti ] = direct_kinematics(PJ_DH)

    oTg = eye(4);

    % Ciclo para percorrer as juntas todas do robot e gerar as
    % respectivas matrizes de transforma��o;
    for i=1:size(PJ_DH,1)

        if PJ_DH(i,6) == 1        % Se a junta � de Rota��o

            thetai = PJ_DH(i,1)+PJ_DH(i,5);
            di = PJ_DH(i,2);
        elseif PJ_DH(i,6) == 0    % Se a junta � Prism�tica

            thetai = PJ_DH(i,1);
            di = PJ_DH(i,2)+PJ_DH(i,5);
        end

        ai = PJ_DH(i,3);
        alfai = PJ_DH(i,4);

        % Par�metros de entrada: thetai di  alfai  ai
        Ti(:,:,i) = jointMatrix(thetai, di, alfai, ai);
        oTg = oTg * Ti(:,:,i);

    end
end