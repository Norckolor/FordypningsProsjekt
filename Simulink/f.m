function f = f(x)
    n = 2;
    f_lower = M(1:n)\( -C( x(1:n),x(n+1:end)) - G(x(1:n)) );

    
    f = [x(1:n)'
        f_lower];
end
