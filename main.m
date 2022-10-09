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

% Description of the contact wall
wall = [-1, 0; % Normal Vector
         3, 0]; % Support Point
% Make sure normal vector is normalized
wall(1,:) = wall(1,:)/norm(wall(1,:));

% Desired Position
q_des = [2; 1; 0;...  % Base
         0; 0; 0; 0]; % Manipulator Joints

% Control Law
u = @(x,t) ctrl(x, q_des, m_base,m_link,l,r); % + (t < 5) * [0;0;0;5] + (t > 5) * [0;0;5;0];

% Dynamic model function
f = @(t, x) am(x, u(x,t), m_base, m_link, r, l,r_tendon, q_des, wall);

% Initial conditions
% ... Base Pose  Manipulator Joints
y0 = [0, 0, 0, 0, 0, 0, 0  ... Integral 
      0, 0, 0, 0, 0, 0, 0 ... Position
      0, 0, 0, 0, 0, 0, 0]; ... Velocity
tspan = 0:0.01:10;

% Simulate system
[t, y] = ode45(f, tspan, y0);

% All plotting
visualize_traj(t,y,q_des,r,l,m_base,m_link,r_tendon,wall)

