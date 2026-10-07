function [f] = f(x,parms)

    % parameters: 
    J1 = parms.J1;
    J2 = parms.J2;
    L1 = parms.L1;
    L2 = parms.L2;
    m1 = parms.m1;
    m2 = parms.m2;
    g = parms.g;
    
    % Intrinsic System values
    n = 2;
        
    % calculated values
    a = J1 + J2 + L1^4/4*(m1 + 4*m2) + L2^2*m2/4;
    b = L1*L2*m2;
    c = J2 + L2^2/4*m2;
    
    % functions
    M = @(q) [a+b*cos(q(2)),    c+b/2*cos(q(2));
              c+b/2*cos(q(2)),  c];
    
    C = @(q,dq) [0,                         -b*(dq(1)+dq(2)/2)*sin(q(2));
                 b*(dq(1)-dq(2)/2)*sin(q(2)), 0];
    
    G = @(q) [g*m2*( L2/2*cos (q(1)+q(2) ) + L1*cos( q(1) ) )+L1*g*m1*cos(q(2));
              L2/2*m2*g*cos( q(1)+q(2) )];
    
    sz = size(x);
    f = zeros(2*n,sz(2));
    for k = 1:sz(2)
        M_ = M( x(1:n,k) );
        C_ = C(x(1:n,k) ,x(n+1:end,k));
        G_ = G(x(1:n,k));
        f_lower = M_\( -C_*x(n+1:end,k) - G_ );
        
        f(:,k) = [x(1:n,k);f_lower];
    end
end
