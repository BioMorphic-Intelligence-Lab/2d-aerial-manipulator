%% Entry Point to the Simulation
% Gets the system parameters and starts the simulation

% Dualrotor Mass
m_base = 0.5; % kg
m_link = 0.01; %kg
% Dualrotor Arm Length
r = 0.1; %m
% Manipulator Link Length
l = 0.1; %m
% Tendon distance to the backbone
r_tendon = 0.05; %m
% Position of the contact wall
x_wall = 3;

% Desired Position
q_des = [3; 1; 0;...  % Base
         0; 0; 0; 0]; % Manipulator Joints

% Control Law
u = @(x) [max(min(angle_lqr(x, m_base, r,...
                          pos_pd(x, m_base, q_des(1)))...
                + height_lqri(x, m_base, r, q_des(2)),...
                [5;5]),...
           0); % Bi-Rotor Inputs
          [-4;0]... Tendon Inputs
            ];

% Dynamic model function
f = @(t, x) am(x, u(x), m_base, m_link, r, l,r_tendon, q_des, x_wall);

% Initial conditions
% ... Base Pose  Manipulator Joints
y0 = [0, 0, 0,   0, 0, 0, 0  ... Integral 
      0, 0, 0,   0, 0, 0, 0 ... Position
      0, 0, 0,   0, 0, 0, 0]; ... Velocity
tspan = 0:0.01:10;

% Simulate system
[t, y] = ode45(f, tspan, y0);

% All plotting
visualize_traj(t,y,u,r,l,m_base,m_link,r_tendon,x_wall)

