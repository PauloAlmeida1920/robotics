function [theta1,theta2] = R4_inverse(P,a_lenght,l)
     % Tf3(1,4).^2+ Tf3(2,4).^2
     %simplify(ans)
     P = P';
    %P(1,1)
    %P(1,2)
    %P(1,3)
    cos_theta2 = sqrt(P(1,1)^2 + P(1,2)^2 - 1);
    sen_theta2 = -P(1,3);
    %check_-------------------------------------------CHCBJBDJH
    theta2 =  atan2( sen_theta2, cos_theta2);
    
    theta1 = atan2(P(1,2),P(1,1)) - atan2(1, cos(theta2)); 
end
