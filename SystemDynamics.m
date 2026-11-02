function dxdt = SystemDynamics(t,x,h,dh,ddh,parms)
    %dxdt = f(x) + g(x)u
    n = 2;
    q = x(1:n);
    dq = x(n+1:end-1);
    s = x(end);
    
    %% path generation and location
    rho = @(s) 1;
    q1_mid = -0.360799276716427;
    q2_mid = 2.296312078050181;
    q_mid = [q1_mid;q2_mid];

    q0 = PathCoordinating(0,h);
    q0_bar = q0 - q_mid;
    
    q_bar = q - q_mid;

    theta = atan2(q_bar(2),q_bar(1));
    theta0 = atan2(q0_bar(2),q0_bar(1));

    if theta < 0 
        theta = theta + 2*pi;
    end
    if theta0 < 0 
        theta0 = theta0 + 2*pi;
    end
    
    theta_bar = -theta + theta0;
    s_ = mod(4*theta_bar/(2*pi),4)
     
    h_ = PathCoordinating(s,h);
    dh_ = PathCoordinating(s,dh);
    ddh_ = PathCoordinating(s,ddh);

    q_r = h_;
    dq_r = dh_*rho(s);
    ddq_r = ddh_*rho(s)^2 + ddh_*0;
    %% controller 
    e = q-h_;
    kp = 0;
    u_transverse = kp*e;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = @(x) u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),u(x),parms);
    dxdt(5) = rho(s);
    dxdt = dxdt';
end