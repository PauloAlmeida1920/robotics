
%% Cria os pontos 3D de cada Objecto
% offset é um vector do tipo [ xoffset yoffset zoffset] que define
% a outra Face paralela a 1a a distância de offset 

function [ points3D ] = createObject(points, offset)


aux = [points(:,1)+offset(1) points(:,2)+offset(2) points(:,3)+offset(3) ];


% Guarda todos os pontos do objecto
points3D = [ points(:,:) aux(:,:) ];

         
end