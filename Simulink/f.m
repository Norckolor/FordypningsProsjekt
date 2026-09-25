function f = f(x)
    
    f_lower(x) = inv(M(x(1:2))*(-C(x(1:2),x(3:4))*q(3:4) - G(x(1:2))));

    f = [x(3);
        x(4);
        f_lower];
end
