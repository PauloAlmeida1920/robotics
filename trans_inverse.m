function trans_inv = trans_inverse(trans)

    R = trans(1:3,1:3); %matriz de rotação do objecto B relativa./ ao mundo A
    P = trans(1:3,4); %vector posição do objecto B original relativa./ ao mundo A
    

    trans_inv = [R' -R'*P;
                   0 0 0 1 ];


end