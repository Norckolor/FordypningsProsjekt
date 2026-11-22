function Df = Df(x,s,parms)
    
    % parameters: 
    J1 = parms.J1;
    J2 = parms.J2;
    L1 = parms.L1;
    L2 = parms.L2;
    m1 = parms.m1;
    m2 = parms.m2;
    grav  = parms.g;
    Path_obj = parms.Path_obj;
    n = Path_obj.dofs;
    q = x(1:n);
    dq = x(n+1:2*n);

    
    dh = Path_obj.dh(s);
    rho = Path_obj.rho(s);
    qr = Path_obj.qr(s);
    e2 = dq - qr;


    M_ = M(q,parms);
    C_ = C(q,dh,parms);
    G_ = G(q,parms);
    
    ddq = @(q,dq) M(q,parms)\(-C(q,dq,parms)-G(q,parms));
    e = 1e-4;
    Ae1 = (ddq(q+e,dq) - ddq(q,dq))/e;
    Ae2 = (ddq(q,dq+e) - ddq(q,dq))/e;
    Df = [zeros(2,2), eye(2,2);
          Ae1, Ae2];
end