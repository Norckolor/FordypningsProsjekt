function f = PathPlanning(p,dp,ddp)
    %%% PathPlanning %%%

    % To compute a path between points we use a 5th. order polynomial:
    % f(x) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
    % To ensure continuity we also need to specify the first and second
    % derivitive at each of the points

    % The return function as in a matrix format:
    % f = [fx1 fx2 fx3...;
    %      fy1 fy2 fy3...];


    % Error in input arguments:
    if ( size(p,2) ~= size(dp,2) )
        error('Different number of points in p to dp')
    end

    if ( size(p,2) ~= size(ddp,2) )
        error('Different number of points in p to ddp')
    end

    %extract x values:
    x = p(1,:);
    dx = dp(1,:);
    ddx = ddp(1,:);
    xp = [x;dx;ddx];

    %extract y values:
    y = p(2,:);
    dy = dp(2,:);
    ddy = ddp(2,:);
    yp = [y;dy;ddy];
    
    N = size(p,2); %number of points

    % A = [1 0 0 0 0 0;
    %      0 1 0 0 0 0;
    %      0 0 1 0 0 0;
    %      1 1 1 1 1 1;
    %      0 1 2 3 4 5;
    %      0 0 2 6 12 20;];

    %precompute inv(A) = A_
    A_ = [1     0   0   0   0   0;
          0     1   0   0   0   0;
          0     0   1   0   0   0;
          -10   -6  -3  10  -4  0.5;
          15    8   3   -15 7   -1 
          -6    -3  -1  6   -3  0.5];
    
    vx = [xp;circshift(xp,[0,-1])];
    ax = zeros(6,N);
    
    vy = [yp;circshift(yp,[0,-1])];
    ay = zeros(6,N);
    
    
    for k = 1:N
        ax(:,k) = A_*vx(:,k);
        ay(:,k) = A_*vy(:,k);
    end
    
    fx = @(s) [1, s, s^2, s^3, s^4, s^5] * ax;
    fy = @(s) [1, s, s^2, s^3, s^4, s^5] * ay;

    

    f = @(s) [fx(s);fy(s)];
end