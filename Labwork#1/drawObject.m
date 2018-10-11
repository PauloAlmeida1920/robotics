
% Função que permite através da matriz dos points XYZ de uma face
% e da uma pose do objecto em relação ao referencial r
% 
% points é uma matriz Nx6 da forma [ x y z x y z ] de duas Faces

function drawObject(points, color)

% Desenho da Face de "Frente" points -> [ x y z ] nx3
fill3(points(:,1), points(:,2), points(:,3), color)
hold on

% Desenho da Face de "Trás" -> Face de "Frente" + offset
aux = [points(:,4) points(:,5) points(:,6) ];

fill3(aux(:,1), aux(:,2), aux(:,3), color)
hold on

% Desenho das faces laterais
for i=1:size(points,1)-1
    
    % ímpar
    ip = mod(i,2);
    if ip == 1
        X = [ points(i,1)' aux(i,1)' aux(i+1,1)' points(i+1,1)' ];
        Y = [ points(i,2)' aux(i,2)' aux(i+1,2)' points(i+1,2)' ];
        Z = [ points(i,3)' aux(i,3)' aux(i+1,3)' points(i+1,3)' ];
    % par
    elseif ip == 0

        X = [ aux(i,1)' points(i,1)' points(i+1,1)' aux(i+1,1)' ];
        Y = [ aux(i,2)' points(i,2)' points(i+1,2)' aux(i+1,2)' ];
        Z = [ aux(i,3)' points(i,3)' points(i+1,3)' aux(i+1,3)' ];
    end

    fill3(X, Y, Z, color)
    hold on 
end

% Desenho da lateral do fundo - caso especial
n = size(points,1);
X = [ points(n,1)' aux(n,1)' aux(1,1)' points(1,1)' ];
Y = [ points(n,2)' aux(n,2)' aux(1,2)' points(1,2)' ];
Z = [ points(n,3)' aux(n,3)' aux(1,3)' points(1,3)' ];

fill3(X, Y, Z, color)
hold on 

% Configuração do gráfico
grid on 
xlabel('Eixo dos xx')
ylabel('Eixo dos yy')
zlabel('Eixo dos zz')
axis([-5 30 -5 30 0 10])

view(140, 30);
set(gca, 'DataAspectRatio', [1 1 1])
           
end