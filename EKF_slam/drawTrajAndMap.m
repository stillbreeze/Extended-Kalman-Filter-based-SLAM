function drawTrajAndMap(x, last_x, P, t)
    %figure(2)
    hold on;
    drawCovEllipse(x(1:2), P(1:2, 1:2), 'b');
    plot([last_x(1) x(1)], [last_x(2) x(2)], 'b');
    plot(x(1), x(2), '*b');
    if t == 0
        for k = 1:6
            drawCovEllipse(x((3 + k*2 - 1):(3 + k*2)), P((3 + k*2 - 1):(3 + k*2), (3 + k*2 - 1):(3 + k*2)), 'r');
        end    
    else
        for k = 1:6
            drawCovEllipse(x((3 + k*2 - 1):(3 + k*2)), P((3 + k*2 - 1):(3 + k*2), (3 + k*2 - 1):(3 + k*2)), 'g');
        end
    end
end