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

qr = zeros(n,N);
for j = 1:N
    qr(:,j) = InvKinematics(c(:,j),parms);
end
qr;

dqr = [ 1, -1;
        1, 1];

ddqr = [1, 1;
        1, 1];

[h,dh,ddh] = PathPlanning(qr,dqr,ddqr);

%% Proof the system works (if i get the implementation right)
% s = 0:0.01:1;
% p = zeros(3,length(s));
% 
% q1 = zeros(N,length(s));
% q2 = zeros(N,length(s));
% figure Name 'statespace'
% hold on
% 
% for j = 1:N
%     h_1 = h{1,j};
%     h_2 = h{2,j};
%     plot(qr(1,j),qr(2,j),'h')
%     for k = 1:length(s)
%         % point(:,k) = ForwardKinematics(h_(s(k)),parms);
%         q1(j,k) = h_1(s(k));
%         q2(j,k) = h_2(s(k));
%     end
%     plot(q1(j,:),q2(j,:))
% 
% end
% legend
% hold off
% 
% figure Name 'xyz'
% x = zeros(N,length(s));
% y = zeros(N,length(s));
% z = zeros(N,length(s));
% hold on
% grid on
% xlim([-2,2])
% ylim([-2,2])
% for j = 1:N
%     for k = 1:length(s)
%         xyz = ForwardKinematics([q1(j,k),q2(j,k)],parms);
%         x(j,k) = xyz(1);
%         y(j,k) = xyz(2);
%         z(j,k) = xyz(3);
% 
% 
%     end
%    plot3(x(j,1),y(j,1),z(j,1),'h')
%    plot3(x(j,:),y(j,:),z(j,:),'LineWidth',2)
% end
% legend
% hold off
%% simulate:


% main_sim
dt = 0.01; 
q0 = InvKinematics(c(:,1),parms);
x0 = [qr(:,1);
      dqr(:,1);
        0];
t = 0:dt:4;
[~,y] = ode45(@(t,x) systemDynamics(t,x,h,dh,ddh,parms),t,x0);
xstart  = y(1,:)
xend    = y(end,:)

theta1 = y(:,1);              % joint 1
theta2 = y(:,2);              % joint 2
%                    % 60 fps visualisation
% 
animate2DOF(parms,theta1,theta2,dt, ...
            'Trace',true, ...
            'FigureName','2‑DOF Robot – sinusoid');

%% functions
function dxdt = systemDynamics(t,x,h,dh,ddh,parms)
    %dxdt = f(x) + g(x)u
    n = 2;
    q = x(1:n);
    dq = x(n+1:end-1);
    s = x(end);
    
    %% path shit
    rho = @(s) 1;
    
    
    h_ = PathCoordinating(s,h);
    dh_ = PathCoordinating(s,dh);
    ddh_ = PathCoordinating(s,ddh);

    q_r = h_;
    dq_r = dh_*rho(s);
    ddq_r = ddh_*rho(s)^2 + ddh_*0;
    %% controller 
    u_transverse = 0;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = @(x) u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),u(x),parms);
    dxdt(5) = rho(s);
    dxdt = dxdt';
end