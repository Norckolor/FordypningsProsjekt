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
%points:
cx = [0    0.5   1   0.5];
cy = [0.5     1   0.5  0];
c = [cx;cy];

% velocities at points: 
vx = [0 1 0 -1];
vy = [1 0 -1 0];
v = [vx;vy];

%accelerations at points:
ax = [1 0 -1 0];
ay = [0 -1 0 1];
a = [ax;ay];

n = 2; %number of DOFs
N = size(c,2); %number of points

qr = zeros(n,N);
dqr = zeros(n,N);
ddqr = zeros(n,N); 
for j = 1:N
    qr(:,j) = InvKinematics(c(:,j),parms);
    dqr(:,j) = InvVelocityKinematics(qr(:,j),v(:,j),parms);
    ddqr(:,j) = InvAccelerationKinematics(qr(:,j),dqr(:,j),a(:,j),parms);
end

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
x0 =    [qr(:,1);
        dqr(:,1);
        0];
t = 0:dt:4;
opt = odeset('RelTol',1e-8);
[~,y] = ode45(@(t,x) systemDynamics(t,x,h,dh,ddh,parms),t,x0,opt);
x_end = y(end,1:5)';
%% plots and animation
%get values of interest
theta1 = y(:,1);              % joint 1
theta2 = y(:,2);              % joint 2

%animate robot arm:
animate2DOF(parms,theta1,theta2,dt, ...
            'Trace',true, ...
            'FigureName','2‑DOF Robot – sinusoid');

%%% plot error:
figure name 'error'
h_ = @(s) PathCoordinating(s,h);
q = zeros(n,length(t));
diff = zeros(n,length(t));
for i = 1:length(t)
    q(:,i) = h_(y(i,5));
    diff(1,i) = theta1(i) - q(1,i);
    diff(2,i) = theta2(i) - q(2,i);
end
subplot(2,1,1)
hold on
grid on
plot(t,diff(1,:),LineWidth=1.5);
legend
hold off

subplot(2,1,2)
hold on
grid on
plot(t,diff(2,:),LineWidth=1.5);
legend
hold off

%%% plot phase planes:
figure name 'q dq'
h_ = @(s) PathCoordinating(s,h);
dh_ = @(s) PathCoordinating(s,dh);
q = zeros(n,length(t));
dq = zeros(n,length(t));
for i = 1:length(t)
    q(:,i) = h_(y(i,5));
    dq(:,i) = dh_(y(i,5));
end

subplot(2,1,1);
hold on
grid on
title('phase plane nominal q_1')
plot(q(1,:),dq(1,:),LineWidth=1.5);
xlabel('\(q_1\)','Interpreter','latex');
ylabel('\(\dot{q}_1\)','Interpreter','latex',Rotation=0);
hold off

subplot(2,1,2);
hold on
grid on
title('phase plane nominal q_2')
plot(q(2,:),dq(2,:),LineWidth=1.5);
xlabel('\(q_2\)','Interpreter','latex');
ylabel('\(\dot{q}_2\)','Interpreter','latex',Rotation=0);
hold off

%%% plot orbit in state space
figure name 'orbit in state space'
q1_mid = sum(q(1,:))/size(q(1,:),2);
q2_mid = sum(q(2,:))/size(q(2,:),2);
hold on
grid on
title('orbit in q_1, q_2 space')
plot(q(1,:),q(2,:),LineWidth=1.5);
plot(q1_mid,q2_mid,'p')
plot(q(1,1),q(2,1),'p')
xlabel('\(q_1\)','Interpreter','latex');
ylabel('\(q_2\)','Interpreter','latex',Rotation=0);
hold off
%% functions
function dxdt = systemDynamics(t,x,h,dh,ddh,parms)
    %dxdt = f(x) + g(x)u
    n = 2;
    q = x(1:n);
    dq = x(n+1:end-1);
    s = x(end);
    
    %% path generation and location
    rho = @(s) 1;
    q1_mid = -0.360799276716427;
    q2_mid = 2.296312078050181;
    q_mid = [q1_mid;q2_mid];

    q0 = PathCoordinating(0,h);
    q0_bar = q0 - q_mid;
    
    q_bar = q - q_mid;

    theta = atan2(q_bar(2),q_bar(1));
    theta0 = atan2(q0_bar(2),q0_bar(1));

    if theta < 0 
        theta = theta + 2*pi;
    end
    if theta0 < 0 
        theta0 = theta0 + 2*pi;
    end
    
    theta_bar = -theta + theta0;
    s_ = mod(4*theta_bar/(2*pi),4)
     
    h_ = PathCoordinating(s,h);
    dh_ = PathCoordinating(s,dh);
    ddh_ = PathCoordinating(s,ddh);

    q_r = h_;
    dq_r = dh_*rho(s);
    ddq_r = ddh_*rho(s)^2 + ddh_*0;
    %% controller 
    e = q-h_;
    kp = 0;
    u_transverse = kp*e;
    u_nom = M(q_r,parms)*ddq_r + C(q_r,dq_r,parms)*dq_r + G(q_r,parms);
    u = @(x) u_nom + u_transverse;

    %% dxdt
    dxdt(1:4) = f(x(1:4),u(x),parms);
    dxdt(5) = rho(s);
    dxdt = dxdt';
end