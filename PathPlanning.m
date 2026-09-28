function f = PathPlanning(p,dp,ddp)

    if ( size(p,2) ~= size(dp,2) )
        error('Different number of points in p to dp')
    end

    if ( size(p,2) ~= size(ddp,2) )
        error('Different number of points in p to ddp')
    end

    x = p(1,:);
    dx = dp(1,:);
    ddx = ddp(1,:);
    xp = [x;dx;ddx];

    y = p(2,:);
    dy = dp(2,:);
    ddy = ddp(2,:);
    yp = [y;dy;ddy];

    N = size(p,2);

    A = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0;
         1 1 1 1 1 1;
         0 1 2 3 4 5;
         0 0 2 6 12 20;];
    
    ax = zeros(6,N);
    vx = [xp;circshift(xp,[0,-1])];

    ay = zeros(6,N);
    vy = [yp;circshift(yp,[0,-1])];

    for k = 1:N
        ax(:,k) = A\vx(:,k);
        ay(:,k) = A\vy(:,k);
    end
    
    fx = @(s) [1, s, s^2, s^3, s^4, s^5] * ax;
    fy = @(s) [1, s, s^2, s^3, s^4, s^5] * ay;

    

    f = @(s) [fx(s);fy(s)];
end