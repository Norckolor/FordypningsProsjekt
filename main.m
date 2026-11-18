clc;clear;
%% add Paths:
addpath Robot_2DOF
addpath Pathplanning
addpath PlotsAndAnimations
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

rho = @(s) 1; %ds/dt
drho = @(s) 0; %drho/ds

pr = zeros(n,N);
dpr = zeros(n,N);
ddpr = zeros(n,N); 
for j = 1:N
    pr(:,j) = InvKinematics(c(:,j),parms);
    dpr(:,j) = InvVelocityKinematics(pr(:,j),v(:,j),parms);
    ddpr(:,j) = InvAccelerationKinematics(pr(:,j),dpr(:,j),a(:,j),parms);
end
Path_obj = PathPlanning(pr,dpr,ddpr,rho,drho);
parms.Path_obj = Path_obj;

%% simulate:

dt = 0.01;
q0 = InvKinematics(c(:,1),parms);
x0 =    [1.2;
        2.4;
        -1.2;
        0];
% x0 = [Path_obj.qr(0);
%       Path_obj.dqr(0)];

t = 0:dt:10;
disp('start sim');
tic
[~,y] = ode45(@(t,x) SystemDynamics(t,x,parms),t,x0);
sim_time = toc;
disp(['end sim in ',num2str(sim_time),'s'])


%% plots and animation
show_animaiton = 0;
show_plots = 1;

if show_animaiton 
    animate2DOF(parms,y,dt, ...
                'Trace',true, ...
                'FigureName','2‑DOF Robot – sinusoid');
end

if show_plots
    plots(parms,y,dt,t)
end