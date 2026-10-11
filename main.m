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

%% simulate:

K = diag([1,1]);
% main_sim
q0 = InvKinematics([0,0.5,0],parms);
x0 = [q0;0;0;0]
t = 0:0.01:10;
[t,y] = ode45(@(t,x) systemDynamics(t,x,h,dh,parms),t,x0);

figure
plot(t,y(:,2))

xend = [y(end,:)];

t      = y(:,5);              % time vector
theta1 = y(:,1);              % joint 1
theta2 = y(:,2);              % joint 2
dt = 0.05;                    % 60 fps visualisation

animate2DOF(parms,theta1,theta2,dt, ...
            'Trace',true, ...
            'FigureName','2‑DOF Robot – sinusoid');

%% functions
function dxdt = systemDynamics(t,x,h,dh,parms)
    %dxdt = f(x) + g(x)u


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
    
    % functions
    M = @(q) [a+b*cos(q(2)),    c+b/2*cos(q(2));
              c+b/2*cos(q(2)),  c];

    %% path shit
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
    %InvVelocityKinematics(x(1:2), dhdt_r(0), parms);

    p = h1_r( x(5) );

    xr1 = InvKinematics(p, parms);
    dxr1 = InvVelocityKinematics(x(1:2), dhdt_r(x(5)), parms);

    xr = [xr1;0;0];
    %% controller 
    kp = 0;
    kd = 50;
    K = diag([kp,kp,kd,kd]);
    E = @(x) x(3:4)'*M(x(1:2))*x(3:4) + m1*grav*0.5*L1*sin(x(1)) + m2*grav*(L1*sin(x(1)) + 0.5*L2*sin(x(1)+x(2)));
    v = -K*(x(1:4))*(E(x) - E([0,pi/4,0,0]')) ; %this does not work
    u = @(x) pinv(g(x(1:4),parms))*( v - f(x(1:4),parms) ); %this works
    
    %% dxdt
    dxdt = f(x(1:4),parms) + g(x(1:4),parms)*u(x);
    dxdt(5) = dsdt( x(5) );
end