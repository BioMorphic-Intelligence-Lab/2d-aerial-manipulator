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
walls = [-1, 0; % Normal Vector
          3, 0; % Support Point
          0, 1; % Normal Vector
          0, 0]; % Support Point
% Make sure normal vector is normalized
walls(1:2:length(walls),:) = walls(1:2:length(walls),:)/...
                             norm(walls(1:2:length(walls),:));

% Desired Position and Force
p_des = @(t) (t <=10) .* [3; 2] + ...
             (t > 10) .* [2; -0.025] + ...
             (t > 15) .* [3.025; 1];
f_des  = [-1; 0];

% Instantiate ctrl instance
c = ctrl();

% Control Law
u = @(x,t) c.pos_ctrl(x, p_des(t), m_base,m_link,r,l); %c.f_ctrl(t,x,p_des(t),f_des,m_base,m_link,r,l,r_tendon);

% Dynamic model function
f = @(t, x) am(x, u(x,t), m_base, m_link, r, l,r_tendon, p_des(t), walls);

% Initial conditions
% ... Base Pose  Manipulator Joints
y0 = [0, 0, 0, 0, 0, 0, 0  ... Integral 
      0, 1, 0, 0, 0, 0, 0 ... Position
      0, 0, 0, 0, 0, 0, 0]; ... Velocity
tspan = 0:0.01:5;

% Simulate system
[t, y] = ode45(f, tspan, y0);

% All plotting
visualize_traj(t,y,u,p_des,r,l,m_base,m_link,r_tendon,walls)

