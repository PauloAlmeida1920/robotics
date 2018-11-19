
%% Função que calcula a matriz Jacobiana de velocidades do "end-effector"
%###################################################################################################
% ARGUMENTOS: 
%       - <oTg> : matriz de transformação simbólica obtida a partir da
%         cinemátic directa dos parâmetros de Denavit-Hartenberg;
%
%       - <Ti> : um conjunto de matrizes elementares obtidas a partir da
%         da aplicão dos parâmetros de Denavit-Hartenberg para cada uma das
%         juntas (1 matriz/junta);
%
%       - <q> : vector que contém os parâmetros do manipulador a controlar,
%         se se tratar de uma junta rotacional o parâmetro será do tipo
%         "theta_" ou se pelo conrtário for uma junta prismáticao
%         parâmetro é da forma "d_";
%
%       - <joint_type> : um vector , contêm 0's e 1's para identificar o tipo
%         de junta do manipulador (rotacional ou prismática);
%
%###################################################################################################

%%
function Jac = Jacobian(oTg, Ti, q, joint_type)

    syms dt
    
    % calcular velocidade linear
   
    % 1º ir buscar as posição através da matriz de transformação do robot 
    % para o gripper:
    
    tx = oTg(1,4);
    ty = oTg(2,4);
    tz = oTg(3,4);
    
    % Jacobiana de velocidades lineares - Matriz Jv
    for i = 1 : size(q,2)
        J_v(1,i) = diff(tx, q(i));
        J_v(2,i) = diff(ty, q(i));
        J_v(3,i) = diff(tz, q(i));
    end   

    % Calcular velocidade angular

    % caso particular: primeira junta
    I = eye(3,3);   % ºRo
    Rot = eye(3,3); % esta matriz é uma matriz de rotação cumulativa entre as juntas do manipulador
    
    J_w(:,1) = I*[0 0 1]' * joint_type(1);  
    
    
    %Para as restantes juntas
    for i = 2 : size(q,2)
        
        Rot = Rot * Ti(1:3, 1:3, i-1); %i=2»0R1, i=3»0R2, i=4»0R3, i=5»0R4, i=6»0R5
        
        J_w = [J_w      Rot * [0 0 1]' * joint_type(i)]; % actualiza a matriz com o valor actual, a multiplicão cumulativa
        
    end
    % se a junta prismática joint_type = 0, o que dá uma velocidade angular nula
    % devolve vector "qsi", contem as duas componentes de velocidade das juntas
    % do robot, velocidade linear e velocidae angular

    Jac = [J_v ; J_w];


end















