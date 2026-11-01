function [ddqdtt] = InvAccelerationKinematics(q,dq,a,parms)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    L1 = parms.L1;
    L2 = parms.L2;
    q1 = q(1);
    q2 = q(2);
    dq1 = dq(1);
    dq2 = dq(2);

    dpdq = [ -L1*sin(q1) - L2*sin( q1+q2 ), -L2*sin( q1+q2 );
             L1*cos(q1) + L2*cos( q1+q2 ), L2*cos( q1+q2 );];
    
    d_dt_dpdq = zeros(2,2);
    d_dt_dpdq(1,1) = -L1*dq1*cos(q1) - L2*(dq1+dq2)*cos(q1+q2);
    d_dt_dpdq(1,2) = -L2*(dq1+dq2)*cos(q1+q2);
    d_dt_dpdq(2,1) = -L1*dq1*sin(q1) - L2*(dq1+dq2)*sin(q1+q2);
    d_dt_dpdq(2,2) = -L2*(dq1+dq2)*sin(q1+q2);

    ddqdtt = dpdq\(a -d_dt_dpdq*dq);
end