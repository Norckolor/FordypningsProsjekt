function [q] = InvKinematics(p,parms)
    %returns the angle of the actuators(q1,q2)
    q = zeros(2,1);
    L1 = parms.L1;
    L2 = parms.L2;
    x = p(1);
    y = p(2);
    z = p(3);
    L = sqrt(x^2 + y^2 + z^2);

    q2 = acos( (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2) );
    q1 = atan2(y,x) - asin( (L2*sin( q2 ))/L );
    q(2) = mod(q2,2*pi);
    q(1) = mod(q1,2*pi);
end