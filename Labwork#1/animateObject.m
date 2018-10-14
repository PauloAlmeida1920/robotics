
%% Anima o objecto M através de do desenho constante do mesmo!

function animateObject(T1, T2, T3, T4)

    global delay pointsM pointsA pointsB pointsC

    hold off
    pause(delay)

    pointsM_ = transPoints(T1, pointsM);
    drawObject(pointsM_, 'r') ;

    pointsA_ = transPoints(T2, pointsA);
     drawObject(pointsA_, 'y');

    pointsB_ = transPoints(T3, pointsB);
    drawObject(pointsB_, 'g');

    pointsC_ = transPoints(T4, pointsC);
    drawObject(pointsC_, 'b');

end