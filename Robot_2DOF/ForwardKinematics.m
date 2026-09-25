function [p] = ForwardKinematics(q,parms)
    %returns the position of the end-efector(x,y,z)
    
    L1 = parms.L1;
    L2 = parms.L2;

    p(1) = L1 * cos( q(1) ) + L2 * cos( q(1)+q(2) );
    p(2) = L1 * sin( q(1) ) + L2 * sin( q(1)+q(2) );
    p(3) = 0;
end