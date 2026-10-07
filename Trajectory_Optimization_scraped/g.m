function g = g(x,u,parms)

    % parameters: 
    J1 = parms.J1;
    J2 = parms.J2;
    L1 = parms.L1;
    L2 = parms.L2;
    m1 = parms.m1;
    m2 = parms.m2;
    
    % Intrinsic System values
    n = 2;
        
    % calculated values
    a = J1 + J2 + L1^4/4*(m1 + 4*m2) + L2^2*m2/4;
    b = L1*L2*m2;
    c = J2 + L2^2/4*m2;
    
    % functions
    M = @(q) [a+b*cos(q(2)),    c+b/2*cos(q(2));
              c+b/2*cos(q(2)),  c];
    
    g_upper = zeros(n,n);
    sz = size(x);
    g = zeros(2*n,sz(2));
    for k = 1:sz(2)
        g_lower = inv( M( x(1:n,k) ) );
        u_ = u(:,k);
        
        g(:,k) = [g_upper;g_lower]*u_;
        %disp(g(:,k))
    end
end
