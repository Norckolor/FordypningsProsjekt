%% Continuous-Time NMPC for a 4-State, 2-Input Robotic Arm System
clear; clc; close all;

%% 1. Define Initial & Final Conditions (4x1)
% State vector x = [q1; q2; dq1; dq2]
xo = 1; yo=1;
[q1_0,q2_0] = InverseKinematics(xo,yo);

xf = 2; yf=2;
[q1_f,q2_f] = InverseKinematics(xf,yf);

x_init  = [q1_0; q2_0; 0; 0];  % Starting state
x_final = [q1_f; q2_f; 0; 0];  % Target state

%% 2. MPC Parameters
nx = 4; % 4 States (2 Positions, 2 Velocities)
nu = 2; % 2 Control Inputs (Torques)
dt = 0.05;         % Sampling time (s)
N  = 50;           % Prediction horizon (steps)
T_sim = 5.0;       % Total simulation time (s)
N_sim = round(T_sim / dt);

% Input Bounds for (N x nu) vector
u_min = -100; 
u_max = 100;
lb = u_min * ones(N, nu);
ub = u_max * ones(N, nu);

% Weighting Matrices
Q = diag([10.0, 10.0, 1.0, 1.0]); % 4x4 State tracking penalty
R = 0.1 * eye(nu);                % 2x2 Control effort penalty

%% 3. Simulation Setup
x_history = zeros(nx, N_sim + 1);
u_history = zeros(nu, N_sim);
x_current = x_init;
x_history(:, 1) = x_current;

U_guess = zeros(N, nu);

% Optimization Options
options = optimoptions('fmincon', ...
    'Display', 'none', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 100);

fprintf('Starting 2D Robotic Arm NMPC Simulation...\n');

%% 4. Receding Horizon Loop
for k = 1:N_sim
    % Objective cost over horizon
    cost_func = @(U) mpc_cost_function(U, x_current, x_final, N, dt, Q, R);
    
    % Terminal constraint: x(N) == x_final
    nonlcon = @(U) mpc_terminal_constraint(U, x_current, x_final, N, dt);
    
    % Solve NLP
    [U_opt, ~] = fmincon(cost_func, U_guess, [], [], [], [], lb, ub, nonlcon, options);
    
    % Apply first control step
    u_apply = U_opt(1, :)'; % Extract 2x1 control vector
    u_history(:, k) = u_apply;
    
    % Step forward
    x_next = rk4_step(@dynamics, x_current, u_apply, dt);
    x_history(:, k+1) = x_next;
    x_current = x_next;
    
    % Warm-start next optimization step
    U_guess = [U_opt(2:end, :); U_opt(end, :)];
    k
end
fprintf('Simulation Complete!\n');


%% 5. Plotting Results
time = 0:dt:T_sim;

figure('Name', '4-State NMPC Results', 'Color', 'w');

% Plot States
subplot(2,1,1);
plot(time, x_history(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'q_1'); hold on;
plot(time, x_history(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'q_2');
yline(x_final(1), 'b--', 'Target q_1');
yline(x_final(2), 'r--', 'Target q_2');
xlabel('Time [s]'); ylabel('Joint Angle [rad]');
title('State Trajectories (rad)'); legend('Location', 'best'); grid on;

% Plot Controls
subplot(2,1,2);
% stairs(time(1:end-1), u_history(1,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'u_1'); hold on;
% stairs(time(1:end-1), u_history(2,:), 'm-', 'LineWidth', 1.5, 'DisplayName', 'u_2');
% yline(u_max, 'k:', 'u_{max}');
% yline(u_min, 'k:', 'u_{min}');
% xlabel('Time [s]'); ylabel('Control Torque [Nm]');
% title('Control Efforts'); legend('Location', 'best'); grid on;

%plot position
x_pos = zeros(1,N_sim);
y_pos = zeros(1,N_sim);
for i = 1:N_sim
    [x_pos(1,i), y_pos(1,i)] = ForwardKinematics(x_history(:,i));
end
plot(x_pos(1,:),y_pos(1,:)); hold on;
title('State Trajectories (Positions)'); legend('Location', 'best'); grid on;

%% =========================================================================
%% HELPER FUNCTIONS
%% =========================================================================

function [x,y] = ForwardKinematics(x)
    l1 = 2;
    l2 = 2;
    abc = x(1);
    def = x(1)+x(2);
    x = l1*cos(abc)+l2*cos(def);
    y = l1*sin(abc)+l2*sin(def);
end

function [q1,q2] = InverseKinematics(x,y)
    l1 = 2;
    l2 = 2;
    
    q2 = acos((x^2 - y^2 - l1^2 - l2^2)/(2*l1*l2));
    q1 = atan2(y,x) - asin(l2*sin(q2)/sqrt(x^2 + y^2));
end

function dxdt = dynamics(x, u)
    dxdt = f(x) + g(x) * u;
end

% RK4 Integration
function x_next = rk4_step(dyn_handle, x, u, dt)
    k1 = dyn_handle(x, u);
    k2 = dyn_handle(x + 0.5*dt*k1, u);
    k3 = dyn_handle(x + 0.5*dt*k2, u);
    k4 = dyn_handle(x + dt*k3, u);
    x_next = x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4);
end

% Cost Function
function cost = mpc_cost_function(U, x0, x_target, N, dt, Q, R)
    cost = 0;
    x_k = x0;
    for i = 1:N
        u_k = U(i, :)';
        x_k = rk4_step(@dynamics, x_k, u_k, dt);
        
        x_err = x_k - x_target;
        cost = cost + x_err' * Q * x_err + u_k' * R * u_k;
    end
end

% Terminal State Equality Constraint
function [c, ceq] = mpc_terminal_constraint(U, x0, x_target, N, dt)
    c = []; 
    x_k = x0;
    for i = 1:N
        u_k = U(i, :)';
        x_k = rk4_step(@dynamics, x_k, u_k, dt);
    end
    ceq = x_k - x_target;
end

% System Autonomous Drift Dynamics f(x)
function [f_x] = f(x)
    J1 = 2; J2 = 2;
    L1 = 2; L2 = 2;
    m1 = 2; m2 = 2;
    g_acc = 9.81;
    
    n = 2;
    q  = x(1:n);
    dq = x(n+1:end);
        
    a = J1 + J2 + (L1^4)/4*(m1 + 4*m2) + (L2^2)*m2/4;
    b = L1*L2*m2;
    c = J2 + (L2^2)/4*m2;
    
    M = [a + b*cos(q(2)),      c + (b/2)*cos(q(2));
         c + (b/2)*cos(q(2)),  c];
    
    C = [0,                         -b*(dq(1) + dq(2)/2)*sin(q(2));
         b*(dq(1) - dq(2)/2)*sin(q(2)), 0];
    
    G = [g_acc*m2*(L2/2*cos(q(1)+q(2)) + L1*cos(q(1))) + L1*g_acc*m1*cos(q(2));
         (L2/2)*m2*g_acc*cos(q(1)+q(2))];
    
    f_lower = M \ (-C*dq - G);
    
    % Kinematics + Dynamics (q_dot = dq)
    f_x = [dq; f_lower];    
end

% System Control Input Matrix g(x)
function [g_x] = g(x)
    J1 = 2; J2 = 2;
    L1 = 2; L2 = 2;
    m1 = 2; m2 = 2;
    
    n = 2;
    q = x(1:n);
    
    a = J1 + J2 + (L1^4)/4*(m1 + 4*m2) + (L2^2)*m2/4;
    b = L1*L2*m2;
    c = J2 + (L2^2)/4*m2;
    
    M = [a + b*cos(q(2)),      c + (b/2)*cos(q(2));
         c + (b/2)*cos(q(2)),  c];
     
    g_x = [zeros(n, n);
           M \ eye(n)];
end