function drawCovEllipse(c, Cov, setting)
    [U D V] = svd(Cov);
    a = D(1,1);
    b = D(2,2);

    vx = U(1,1);
    vy = U(1,2);


    th = atan2(vy, vx);
    R = [cos(th) -sin(th) ; sin(th) cos(th)];
    
    
    phi = [0:(pi/50):(2*pi)];
    
    rot = zeros(2,101);
    
    for i = 1:101
        rect = [3*sqrt(a)*cos(phi(i)) ; 3*sqrt(b)*sin(phi(i))];
        rot(:,i) = R*rect + c;    
    end
    plot(rot(1,:), rot(2,:), setting);
    axis equal;
end