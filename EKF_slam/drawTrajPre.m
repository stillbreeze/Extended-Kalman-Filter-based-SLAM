function drawTrajPre(x, P)
    %figure(2)
    hold on;
    drawCovEllipse(x(1:2), P(1:2, 1:2), 'm');
    
end