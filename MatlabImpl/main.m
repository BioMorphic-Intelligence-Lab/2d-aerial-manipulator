%% Entry Point to the Simulation
% Gets the system parameters and starts the simulation

% Dualrotor Mass
m_base = 0.5; % kg
m_link = 0.1; %kg
% Dualrotor Arm Length
r = 0.1; %m
% Manipulator Link Length
l = 0.1; %m

% Desired Position
q_des = [3; 1; 0;...  % Base
         0; 0; 0; 0]; % Manipulator Joints

% Control Law
u = @(x) max(min(angle_lqr(x, m_base, r,...
                          pos_pd(x, m_base, q_des(1)))...
                + height_lqri(x, m_base, r, q_des(2)),...
                [5;5]),...
            0);

% Dynamic model function
f = @(t, x) am(x, u(x), m_base, m_link, r, l, q_des);

% Initial conditions
% ... Base Pose  Manipulator Joints
y0 = [0, 0, 0,   0, 0, 0, 0  ... Integral 
      0, 0, 0,   0.1, 0, 0, 0 ... Position
      0, 0, 0,   0, 0, 0, 0]; ... Velocity
tspan = 0:0.01:15;

% Simulate system
[t, y] = ode45(f, tspan, y0);

% All plotting
visualize_traj(t,y,u,r,l)

