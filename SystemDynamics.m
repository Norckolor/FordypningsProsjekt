function dxdt = SystemDynamics(t,x,qr,dqr,ddqr,rho,parms)
    %dxdt = f(x) + g(x)u
    n = 2;
    q = x(1:n);
    dq = x(n+1:end-1);
    %s = x(end);
    dt = 0.01;
    orbit_length = 2*pi;
    %% path generation and location
    
    %displacement of orbit so atan2 is used in the center of the orbit
    q_mid = 0;
    for s = 0:dt:orbit_length
        temp = qr(s); 
        q_mid = q_mid + dt*temp;
    end
    q_mid = q_mid/(2*pi);
    q_bar = q - q_mid;

    % rotation of orbit so 0 of atan2 and path lines up
    qr0 = qr(0);
    qr0_bar = qr0 - q_mid;
    theta0 = atan2(qr0_bar(2),qr0_bar(1));
    
    
    theta = atan2(q_bar(2),q_bar(1));
    theta_bar = theta-theta0;

    s = mod(theta_bar,2*pi);
     
    % h_ = PathCoordinating(s,h);
    % dh_ = PathCoordinating(s,dh);
    % ddh_ = PathCoordinating(s,ddh);
    q_r = qr(s);
    dq_r = dqr(s);
    ddq_r = ddqr(s);
    %% controller 
    e = q-q_r;
    de = dq-dq_r;
    kp = 0; kd = 0;
    u_transverse = kp*e + kd*de;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = @(x) u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),u(x),parms);
    dxdt = dxdt';
end