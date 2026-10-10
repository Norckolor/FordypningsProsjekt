clc;clear;
%% add Paths:
addpath Robot_2DOF
addpath Pathplanning


%% set system paramters:
parms.J1 = 2;
parms.J2 = 2;
parms.L1 = 2;
parms.L2 = 2;
parms.m1 = 2;
parms.m2 = 2;
parms.g = 9.81;


%% Generate path:

pathplanEx
% Path




%% simulate:

%main_sim
x0 = [0,0,0,0,0];
t = 0:0.01:5;
ode45(@(t,x) noe(t,x,h,dh,parms),t,x0)

%% functions
function dxdt = noe(t,x,h,dh,parms)
     %dxdt = f(x) + g(x)u
    h_1 = h{1,1};
    h_2 = h{2,1};
    h_3 = h{3,1};
    
    % dPath/ds
    dhds_1 = dh{1,1};
    dhds_2 = dh{2,1};
    dhds_3 = dh{3,1};
    rho = @(s) 1;

    dsdt = @(s) rho(s);
    dhdt_1 = @(s) dhds_1(s)*dsdt(s);
    dhdt_2 = @(s) dhds_2(s)*dsdt(s);
    dhdt_3 = @(s) dhds_3(s)*dsdt(s);
    
    h1_r = @(s) [h_1(s);h_2(s);h_3(s)];
    dhdt_r = @(s) [dhdt_1(s);dhdt_2(s);dhdt_3(s)];
    InvVelocityKinematics(x(1:2), dhdt_r(0), parms);

    p = h1_r( x(5) );

    xr1 = InvKinematics(p, parms);
    dxr1 = InvVelocityKinematics(x(1:2), dhdt_r(x(5)), parms);

    xr = [xr1;dxr1];

    kp = 1;
    kd = 1;
    K = diag([kp,kp,kd,kd]);
   
    v = -K(x(1:4)-xr);

    u = g(x(1:4))\(v-f(x(1:4)));

    dxdt = f(x,parms) + g(x,u,parms);
    dxdt(5) = dsdt( x(5) );
end