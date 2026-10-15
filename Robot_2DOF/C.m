function [C] = C(q,dq,parms)
    % parameters: 
    J1 = parms.J1;
    J2 = parms.J2;
    L1 = parms.L1;
    L2 = parms.L2;
    m1 = parms.m1;
    m2 = parms.m2;
    grav  = parms.g;

    % calculated values
    a = J1 + J2 + L1^4/4*(m1 + 4*m2) + L2^2*m2/4;
    b = L1*L2*m2;
    c = J2 + L2^2/4*m2;

    C = [0,                         -b*(dq(1)+dq(2)/2)*sin(q(2));
         b*(dq(1)-dq(2)/2)*sin(q(2)), 0];
end