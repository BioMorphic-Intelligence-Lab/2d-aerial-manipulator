%% Entry Point to the Simulation
% Gets the system parameters and starts the simulation

% Init struct for logging
global logger
logger.t = [];%;zeros(1,1);
logger.u = [];%zeros(4,1);
logger.q = [];%zeros(7,1);
logger.qdot = [];%zeros(7,1);
logger.ref = [];%zeros(2,1);
logger.fext = [];%zeros(2,1);
logger.fext_est = [];%zeros(2,1);


% Dualrotor Mass
m_base = 0.5; % kg
m_link = 0.01; %kg
% Dualrotor Arm Length
r = 0.1; %m
% Manipulator Link Length
l = 0.1; %m
% Tendon distance to the backbone
r_tendon = 0.05; %m
% Maximal thrust of one thruster
u_max = 5; %N

% Description of the contact wall
walls = [-1, 0; % Normal Vector
          3, 0; % Support Point
          0, 1; % Normal Vector
          0, 0]; % Support Point
% Make sure normal vector is normalized
walls(1:2:length(walls),:) = walls(1:2:length(walls),:)/...
                             norm(walls(1:2:length(walls),:));

% Desired Position and Force
dir = [3.25; 1];
f_des  = [-0.2; 0];

% Instantiate ctrl instance
c = ctrl();

% Control Law
u = @(x,t) c.f_ctrl(t,x,dir, f_des,m_base,m_link,r,l,r_tendon,u_max);
%u = @(x,t) c.pos_ctrl(x,dir,m_base,m_link,r,l);
% Dynamic model function
f = @(t, x) am(t,x, u(x,t), m_base, m_link, r, l,r_tendon, dir, walls);

% Initial conditions
% ... Base Pose  Manipulator Joints
y0 = [0, 0, 0, 0, 0, 0, 0  ... Integral 
      2, 1, 0, 0, 0, 0, 0 ... Position
      0, 0, 0, 0, 0, 0, 0]; ... Velocity
tspan = 0:0.01:10;

% Simulate system
[t, y] = ode45(f, tspan, y0);

%u = @(x,t) c.pos_ctrl(x,dir,m_base,m_link,r,l);
% All plotting
visualize_traj(logger,f_des,r,l,walls,u_max)

