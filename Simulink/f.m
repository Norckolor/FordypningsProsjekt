function [f] = f(x)

    %%% system values 
    J1 = 2;
    J2 = 2;
    L1 = 2;
    L2 = 2;
    m1 = 2;
    m2 = 2;
    g = 9.81;
    
    %%% Intrinsic System values
    n = 2;
        
    %%% calculated values
    a = J1 + J2 + L1^4/4*(m1 + 4*m2) + L2^2*m2/4;
    b = L1*L2*m2;
    c = J2 + L2^2/4*m2;
    
    %%% functions
    M = @(q) [a+b*cos(q(2)),    c+b/2*cos(q(2));
              c+b/2*cos(q(2)),  c];
    
    C = @(q,dq) [0,                         -b*(q(1)+q(2)/2)*sin(q(2));
                 b*(q(1)-q(2)/2)*sin(q(2)), 0];
    
    G = @(q) [g*m2*(L2/2*cos(q(1)+q(2)+L1*cos(q(1))))+L1*g*m1*cos(q(2));
              L2/2*m2*g*cos(q(1)+q(2))];

    n = 2;
    f_lower = M(x(1:n))\( -C(x(1:n) ,x(n+1:end))*x(n+1:end) - G(x(1:n)) );

    
    f = [x(1:n);
        f_lower];
end
