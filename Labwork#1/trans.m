%____________________________________________________________________________________    
% T = trans(alpha, beta, gama, pos)
% onde:
% alpha, beta e gama = ângulos de EULER Z-Y-X (alpha?beta?gama)
% pos  = [tx ty tz] - vector da posição que relaciona a posição do referencial objecto
% (B) com o referencial mundo (A)
%____________________________________________________________________________________

function T = trans(alpha, beta, gama, pos)

    alpha = deg2rad(alpha);
    beta = deg2rad(beta);
    gama = deg2rad(gama);
    
    R = euler_rot(alpha, beta, gama);
    T = [R pos;
        0 0 0 1];
end
    
