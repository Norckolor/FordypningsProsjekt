function [h,dh,ddh] = PathPlanning(p,dp,ddp)
    %%% PathPlanning %%%

    % To compute a path between points we use a 5th. order polynomial:
    % f(x) = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
    % To ensure continuity we also need to specify the first and second
    % derivitive at each of the points

    % The return function as in a cell format:
    % f = [fx1 fx2 fx3...;
    %      fy1 fy2 fy3...
    %      .    .   .   
    %      .    .   .
    %      .    .   .];


    % Error in input arguments:
    if ( size(p,2) ~= size(dp,2) )
        error('Different number of points in p to dp')
    end

    if ( size(p,2) ~= size(ddp,2) )
        error('Different number of points in p to ddp')
    end
    
    N = size(p,2); %number of points (index j)
    n = size(p,1); %number of DOFS  (index i)
    
    vx = zeros(6,N,n);
    for i = 1:n 
        x = p(i,:);
        dx = dp(i,:);
        ddx = ddp(i,:);
        xp = [x;dx;ddx];

        vx(:,:,i) = [xp;circshift(xp,[0,-1])];
    end

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
    

    ax  = zeros(6,N,n);
    h   = cell(n,N);
    dh  = cell(n,N);
    ddh = cell(n,N);

    for i = 1:n
        for j = 1:N 
        
        ax(:,i) = A_*vx(:,j,i);
        
        h{i,j} = @(s) [1, s, s^2, s^3, s^4, s^5] * ax(:,i);
        dh{i,j} = @(s) [1, 2*s, 3*s^2, 4*s^3, 5*s^4] * ax(2:end,i);
        ddh{i,j} = @(s) [2, 6*s, 12*s^2, 20*s^3] * ax(3:end,i);
        end
    end
end