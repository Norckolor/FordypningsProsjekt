function Path_obj = MakePath(parms)

    rho = parms.rho;
    drho = parms.drho;

    %points:
    cx = [0    0.5   1   0.5];
    cy = [0.5     1   0.5  0];
    c = [cx;cy];
    
    % velocities at points:  
    vx = [0 1 0 -1];
    vy = [1 0 -1 0];
    v = [vx;vy];
    
    %accelerations at points:
    ax = [1 0 -1 0];
    ay = [0 -1 0 1];
    a = [ax;ay];
    
    n = 2; %number of DOFs
    N = size(c,2); %number of points
    
    pr = zeros(n,N);
    dpr = zeros(n,N);
    ddpr = zeros(n,N); 
    for j = 1:N
        pr(:,j) = InvKinematics(c(:,j),parms);
        dpr(:,j) = InvVelocityKinematics(pr(:,j),v(:,j),parms);
        ddpr(:,j) = InvAccelerationKinematics(pr(:,j),dpr(:,j),a(:,j),parms);
    end
    Path_obj = PathPlanning(pr,dpr,ddpr,rho,drho);
    
end