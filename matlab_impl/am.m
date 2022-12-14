function am = am(t,x,u, m_base, m_link, r, l,r_tendon, p_des, walls)
%AM Computes the state derivative given the current state andc nput
%   The state consists of the dualrotor pose and velocities
%   x is the current state
%   u is the current control input
%   m = m_link + m_base is the system weight
%   r rotor arm length
%   l is each individual arm link length
%   r_tendon is the distance of the tendon to the backbone
%            (needed for torque calculation on the individual joints)
%   q_des desired state
%   wall is the description of the contact wall

global logger

% Log control and time
logger.t = [logger.t,t];
logger.u = [logger.u,u];

% Extract pose and velocities
q_base = reshape(x(8:10),[3,1]);
q_mani = reshape(x(11:14),[4,1]);
q_dot_base = reshape(x(15:17),[3,1]);
q_dot_mani = reshape(x(18:21),[4,1]);
logger.q = [logger.q,[q_base;q_mani]];
logger.qdot = [logger.qdot,[q_dot_base;q_dot_mani]];
 
% Initialize the derivative vector
x_dot = zeros(21,1);

% The rate of change of the integral is just the value (difference)
x_dot(1:2) = q_base(1:2) - p_des;
x_dot(3) = q_base(3);
% It's a first order system so the pose derivative is just the velocities
x_dot(8:14) = [q_dot_base; q_dot_mani];

% Define the equation of motion matrices
[M, C, A, D, K, G] = eom_matrices([q_base;q_mani], [q_dot_base;q_dot_mani],...
                            m_base, m_link, r, l,r_tendon);

% Find the external force
f_ext = contact_dynamics([q_base;q_mani],[q_dot_base;q_dot_mani],m_base,...
                         m_link,r,l,walls);
logger.fext = [logger.fext,f_ext];

% Mapping Jacobian from external force to generalized forces
J_ext = EE_Jacobian([q_base;q_mani],l);

% The velocities derivatives (accelerations) are computed via the equations
% of motion of the dualrotor
x_dot(15:21) = M\(A * u - C*[q_dot_base;q_dot_mani] ...
                  - D*[q_dot_base; q_dot_mani] - K - G ...
                  + virtual_force([q_base;q_mani],u,m_base,m_link,r,l)...
                  + J_ext'*f_ext);

% Store the derivative in the return value
am = x_dot;
end

