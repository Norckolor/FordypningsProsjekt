%% add Paths:
addpath Robot_2DOF\
addpath Pathplanning\


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

main_sim