%% Entry Point to the Simulation
% Gets the system parameters and starts the simulation

% Dualrotor Mass
m_rot = 0.5; % kg
% Dualrotor Arm Length
r = 0.1; %m

% Desired Position
p_des = [-3; 0; 0];

% Control Law
u = @(x) max(min(angle_lqr(x, m_rot, r,...
                           pos_lqr(x, m_rot, p_des(1)))...
                 + height_lqri(x, m_rot, r, p_des(2)),...
                 [5;5]),...
             0);

% Dynamic model function
f = @(t, x) am(x, u(x), m_rot, r, p_des);

% Initial conditions
y0 = [0, 0, 0,  ... Integral 
      0, 0, 0,  ... Position
      0, 0, 0]; ... Velocity
tspan = 0:0.01:10;

% Simulate system
[t, y] = ode45(f, tspan, y0);

% All plotting
visualize_traj(t,y,u)

