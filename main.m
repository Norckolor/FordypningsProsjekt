clc;clear;
%% add Paths:
addpath Robot_2DOF
addpath Pathplanning
addpath PlotsAndAnimations
addpath GeneratePath
addpath Kontroller_info
%% set system paramters:
parms.J1 = 2;
parms.J2 = 2;
parms.L1 = 1;
parms.L2 = 1;
parms.m1 = 2;
parms.m2 = 2;
parms.g = 9.81;
parms.rho = @(s) 1; %ds/dt
parms.drho = @(s) 0; %drho/ds

%% Generate path:
parms.Path_obj = MakePath(parms);

%% Save parameters for future use:
save("parms.mat","parms");
%% Make controller:
tic
% [K,Rfun,Info,C0v,Civ,Div] = MakeController(parms);
% save("Kontroller_info\K.mat","K")
% save("Kontroller_info\Rfun.mat","Rfun")
% save("Kontroller_info\info.mat","Info")
% save("Kontroller_info\C0v.mat","C0v")
% save("Kontroller_info\Civ.mat","Civ")
% save("Kontroller_info\Div.mat","Div")
load("Kontroller_info\K.mat") %made with order = 50, l = 100,kappa = 1, Q = eye(n),Gamma = eye(m), took ca 1h 30min to run (a few min more, no reason to count it)
toc
%% simulate:

dt = 0.01;
x0 =    [1.2;
        2.4;
        -1.2;
        0];
% x0 = [parms.Path_obj.qr(0);
%       parms.Path_obj.dqr(0)];

t = 0:dt:20;
opt = odeset('RelTol',1e-7);
disp('start sim');
tic
[~,y] = ode45(@(t,x) SystemDynamics(t,x,K,parms),t,x0,opt);
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