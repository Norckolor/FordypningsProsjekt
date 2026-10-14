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
cx = [0 0.5 1 0.5];
cy = [0.5 1 0.5 0];
cz = [0 0 0 0];
c = [cx;cy;cz];
n = 2;
N = size(c,2);

q_r = zeros(n,N);
for j = 1:N
    q_r(:,j) = InvKinematics(c(:,j),parms);
end
p = q_r;
   for j = 1:N
        if j == 1 
            last_p = zeros(n,1);
            last_dp = zeros(n,1);
        else
            last_p = p(:,j-1);
            last_dp = dp(:,j-1);
        end
        dp(:,j) = p(:,j) - last_p;
        ddp(:,j) = dp(:,j) - last_dp;
    end

[h,dh,ddh] = PathPlanning(p,dp,ddp);

%% simulate:


% main_sim
q0 = InvKinematics(c(:,1),parms);
x0 = [q0;0;0];
t = 0:0.01:10;
[t,y] = ode45(@(t,x) systemDynamics(t,x,h,dh,ddh,parms),t,x0);
xend = y(end,:)
figure
plot(t,y(:,2))'

xend = [y(end,:)];

t      = y(:,5);              % time vector
theta1 = y(:,1);              % joint 1
theta2 = y(:,2);              % joint 2
dt = 0.05;                    % 60 fps visualisation

animate2DOF(parms,theta1,theta2,dt, ...
            'Trace',true, ...
            'FigureName','2‑DOF Robot – sinusoid');

%% functions
function dxdt = systemDynamics(t,x,h,dh,ddh,parms)
    %dxdt = f(x) + g(x)u

    % parameters: 
    J1 = parms.J1;
    J2 = parms.J2;
    L1 = parms.L1;
    L2 = parms.L2;
    m1 = parms.m1;
    m2 = parms.m2;
    grav  = parms.g;

    %% path shit

    q_r = h(s);
    dq_r = dh(s)*rho(s);
    ddq_r = ddh(s)*rho(s)^2 + ddh(s)*0;
    %% controller 
    u_transverse = 0;
    u_nom = M(q_r)*ddq_r + C(q_r,dq_r)*dq_r + G(q_r);
    u = @(x) u_nom(x) + u_transverse;

    %% dxdt
    dxdt = f(x(1:4),parms) + g(x(1:4),parms)*u(x);
end