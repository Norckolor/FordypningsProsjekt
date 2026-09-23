function g = g(x)

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
    
    g_lower = inv(M(x(1:n)));
    g_upper = zeros(n,n);
    g = [g_upper;g_lower];
end
