function dxdt = SystemDynamics(t,x,qr,dqr,ddqr,rho,parms)
    %dxdt = f(x) + g(x)u
    n = 2;
    q = x(1:n);
    dq = x(n+1:end-1);
    %s = x(end);
    
    %% path generation and location
    
    %sum(h)/periodetid
    theta = atan2(q(2),q(1));

    s = mod(theta,2*pi)
     
    % h_ = PathCoordinating(s,h);
    % dh_ = PathCoordinating(s,dh);
    % ddh_ = PathCoordinating(s,ddh);
    q_r = qr(s);
    dq_r = dqr(s);
    ddq_r = ddqr(s);
    %% controller 
    e = q-q_r;
    de = dq-dq_r/rho(s);
    kp = 0; kd = 0;
    u_transverse = kp*e + kd*de;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = @(x) u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),u(x),parms);
    dxdt = dxdt';
end