function [f] = f(x,u,parms)
    % Intrinsic System values
    n = 2;
    q = x(1:n);
    dq = x(n+1:2*n);

   
    % functions
    M_ = M(q,parms);
    C_ = C(q,dq,parms);
    G_ = G(q,parms);
    
    
    f_lower = M_ \ ( u - C_*dq - G_ );
    
    f = [dq;f_lower];
end
