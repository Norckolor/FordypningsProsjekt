function dxdt = SystemDynamics(t,x,path_obj,parms)
    %% extract values from funciton input:
    n = 2;
    q = x(1:n);
    dq = x(n+1:2*n);

    qr_mid = path_obj.qr_mid;
    qr = @(s) path_obj.qr(s);
    dqr = @(s) path_obj.dqr(s);
    ddqr = @(s) path_obj.ddqr(s);
    path_length = path_obj.length;
    
    persistent x0;
    if isempty(x0)
        x0 = 0;
    end
    %% Projection:

    %%% With atan2 :
    %finding theta offset
    % qr0_bar = qr(0) - qr_mid;
    % theta0 = atan2(qr0_bar(2),qr0_bar(1));
    % theta0 = mod(theta0,2*pi);
    % 
    % q_bar = q-qr_mid; 
    % theta = atan2(q_bar(2),q_bar(1));
    % theta = mod(theta,2*pi);
    % 
    % theta_bar = mod(-(theta-theta0),2*pi);
    % 
    % s = path_length*theta_bar/(2*pi); 
    
    %%% with optimization
    A = eye(n);

    %(x-xr)'A(x-xr)
    %func = @(s) (q-qr(s))'*A*(q-qr(s)) + (dq-dqr(s))'*A*(dq-dqr(s));
    
    %(q-qr)'A(q-qr)
    func = @(s) (q-qr(s))'*A*(q-qr(s)); 

    opts = optimoptions('fminunc','Display','off');
    s = fminunc(func,x0,opts);
    x0 = s;

    %%% use calculated s value 
    q_r = qr(s);
    dq_r = dqr(s);
    ddq_r = ddqr(s);

    %% controller 
    e = q-q_r;
    de = dq-dq_r;
    kp = -30; kd = -30; %-10 -10 looked nice ish
    u_transverse = kp*e + kd*de;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),u,parms);
    dxdt = dxdt';
end