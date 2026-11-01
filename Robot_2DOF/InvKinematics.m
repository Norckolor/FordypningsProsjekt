function [q] = InvKinematics(p,parms)
    %returns the angle of the actuators(q1,q2)
    q = zeros(2,1);
    L1 = parms.L1;
    L2 = parms.L2;
    x = p(1);
    y = p(2);
    L = sqrt(x^2 + y^2);

    q2 = acos( (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2) );
    q1 = atan2(y,x) - atan2((L2*sin(q2)),L1+L2*cos(q2) );

    q(2) =  mod( q2 + pi, 2 * pi ) - pi;
    q(1) =  mod( q1 + pi, 2 * pi ) - pi;
end