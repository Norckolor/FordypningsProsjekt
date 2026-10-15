clc;clear;
%% add Paths:
addpath Robot_2DOF
addpath Pathplanning


%% set system paramters:
parms.J1 = 2;
parms.J2 = 2;
parms.L1 = 1;
parms.L2 = 1;
parms.m1 = 2;
parms.m2 = 2;
parms.g = 9.81;


%% Generate path:
cx = [0     0.5];
cy = [0.5   1];
cz = [0     0];
c = [cx;cy;cz];
n = 2;
N = size(c,2);

q_r = zeros(n,N);
for j = 1:N
    q_r(:,j) = InvKinematics(c(:,j),parms);
end
p = q_r
dp = [0,1;1,0;0,0];
ddp = [1,1;1,1;1,1];

[h,dh,ddh] = PathPlanning(p,dp,ddp);

% h_ = @(s) [h_1(s);h_2(2)];
% h_(1)
% oo = ForwardKinematics(h_(0),parms)
% uu = ForwardKinematics(h_(1),parms)
s = 0:0.01:1;
p = zeros(3,length(s));

q1 = zeros(N,length(s));
q2 = zeros(N,length(s));

hold on

for j = 1:N
    h_1 = h{1,j};
    h_2 = h{2,j};
    plot(q_r(1,j),q_r(2,j),'h')
    for k = 1:length(s)
        % point(:,k) = ForwardKinematics(h_(s(k)),parms);
        q1(j,k) = h_1(s(k));
        q2(j,k) = h_2(s(k));
    end
    plot(q1(j,:),q2(j,:))
    
end
hold off
% hold on
% plot(p(1,1),p(2,1),'p')
% plot(q1,q2,'Color','r')
% plot(p(1,2),p(2,2),'h')
% 
% plot(point(1,1),point(2,1),'p')
% plot(point(1,:),point(2,:),'color','b')
% plot(point(1,end),point(2,end),'h')
% hold off
%% simulate:


% main_sim
% dt = 0.01; 
% q0 = InvKinematics(c(:,1),parms);
% x0 = [q0;0;0;0];
% t = 0:dt:1;
% [t,y] = ode45(@(t,x) systemDynamics(t,x,h,dh,ddh,parms),t,x0);
% xend = y(end,:)
% figure
% plot(t,y(:,2))
% 
% xend = [y(end,:)];
% 
% t      = y(:,5);              % time vector
% theta1 = y(:,1);              % joint 1
% theta2 = y(:,2);              % joint 2
%                    % 60 fps visualisation
% 
% animate2DOF(parms,theta1,theta2,dt, ...
%             'Trace',true, ...
%             'FigureName','2‑DOF Robot – sinusoid');

%% functions
function dxdt = systemDynamics(t,x,h,dh,ddh,parms)
    %dxdt = f(x) + g(x)u
    n = 2;
    q = x(1:n);
    dq = x(n+1:end-1);
    s = x(end);
    
    % parameters: 
    J1 = parms.J1;
    J2 = parms.J2;
    L1 = parms.L1;
    L2 = parms.L2;
    m1 = parms.m1;
    m2 = parms.m2;
    grav  = parms.g;

    %% path shit
    rho = @(s) 0.5;
    h_1 = h{1,1};
    dh_1 = dh{1,1};
    ddh_1 = ddh{1,1};
    
    h_2 = h{2,1};
    dh_2 = dh{2,1};
    ddh_2 = ddh{2,1};
    
    h_ = [h_1(s);h_2(s)];
    dh_ = [dh_1(s);dh_2(s)];
    ddh_ = [ddh_1(s);ddh_2(s)];

    q_r = h_;
    dq_r = dh_*rho(s);
    ddq_r = ddh_*rho(s)^2 + ddh_*0;
    %% controller 
    u_transverse = 0;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = @(x) u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),parms) + g(x(1:4),parms)*u(x);
    dxdt(5) = rho(s);
    dxdt = dxdt'
end