function [q] = InvKinematics(p,parms)
    %returns the angle of the actuators(q1,q2)

    L1 = parms.L1;
    L2 = parms.L2;
    x = p(1);
    y = p(2);
    z = p(3);
    L = sqrt(x^2 + y^2 + z^2);

    q(2) = acos( (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2) );
    q(1) = atan2(y,x) - asin( (L2*sin( q(2) ))/L );
end