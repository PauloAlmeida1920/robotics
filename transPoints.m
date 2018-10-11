
%% Transformação dos pontos para uma nova posição

function newPoints = transPoints(T, oldPoints)

    for i=1:size(oldPoints,1)
        % Matriz 4x4 * Vector 1x4
        newPoints_ = T*[oldPoints(i,1:3) 1]';
        aux = T*[oldPoints(i,4:6) 1]';        
        newPoints(i,:) = [newPoints_(1:3)' aux(1:3)'];
    end

end