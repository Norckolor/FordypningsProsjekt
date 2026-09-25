%% path planning based on Matthew Peter Kelly
% Addapted to this system
clear; clc; close all;
addpath OptimTraj-master\
addpath Robot_2DOF\

%% system Parameters
parm.J1 = 2;
parm.J2 = 2;
parm.L1 = 2;
parm.L2 = 2;
parm.m1 = 2;
parm.m2 = 2;
parm.g = 9.81;


%% Path parameters

p_init = [0,4,0];
q_init = InvKinematics(p_init,parm)';
dq_init = [0,0]';
x_init = [q_init;dq_init];

p_final = [2,2,0]; 
q_final = InvKinematics(p_final,parm)';
dq_final = [0,0]';
x_final = [q_final;dq_final];

duration = 10;
maxTorque = 100;

%% Optimization paramters
problem.func.dynamics = @(t,x,u) f(x,parm) + g(x,u,parm);
problem.func.pathObj = @(t,x,u)( u.^2 + x(3:4,:).^2 );  %torque-squared cost function

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = x_init;
problem.bounds.initialState.upp = x_init;
problem.bounds.finalState.low = x_final;
problem.bounds.finalState.upp = x_final;

problem.bounds.state.low = [-pi;-pi;-10;-10];
problem.bounds.state.upp = [pi;pi;100;100];

problem.bounds.control.low = -[maxTorque;maxTorque];
problem.bounds.control.upp = [maxTorque;maxTorque];

%init guess:
problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [[0;0],[0;0]];

%solver options
problem.options.nlpOpt = optimset(...
    'Display','iter',...
    'MaxFunEvals',1e5);
problem.options.method = 'rungeKutta';


%% Solve

soln = optimTraj(problem);


%%
q_sol = soln.grid.state;
time_sol = soln.grid.time;
pos_sol = zeros(3,length(time_sol));
for k = 1:length(time_sol)
    pos_sol(:,k) = ForwardKinematics(q_sol(:,k),parm);
end

plot(pos_sol(1,:),pos_sol(2,:))
