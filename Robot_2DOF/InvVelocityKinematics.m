function [dqdt] = InvVelocityKinematics(q,v,parms)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    L1 = parms.L1;
    L2 = parms.L2;
    dpdq = [ -L1*sin(q(1)) - L2*sin( q(1)+q(2) ), -L2*sin( q(1)+q(2) );
             L1*cos(q(1)) + L2*cos( q(1)+q(2) ), L2*cos( q(1)+q(2) );];
    
    dqdt = inv(dpdq)*v(1:2);
end