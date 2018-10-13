
%% Matriz de rotação através do vector r e angulo phi  
% Slides III p.16

function [ R ] = vectorRot(r, phi)

    %r = r';
    v = 1 - cos(phi);
    
    % Normalização do Vector de rotação:
    rx = r(1,1) / sqrt( r(1,1)^2 + r(1,2)^2 + r(1,3)^2 );
    ry = r(1,2) / sqrt( r(1,1)^2 + r(1,2)^2 + r(1,3)^2 );
    rz = r(1,3) / sqrt( r(1,1)^2 + r(1,2)^2 + r(1,3)^2 );
    
    s = sin(phi);
    c = cos(phi);
    
    R = [ rx^2*v+c         rx*ry*v-rz*s    rx*rz*v+ry*s;
          rx*ry*v+rz*s     ry^2*v+c        ry*rz*v-rx*s;
          rx*rz*v-ry*s     ry*rz*v+rx*s         rz^2*v+c  ];
   
end