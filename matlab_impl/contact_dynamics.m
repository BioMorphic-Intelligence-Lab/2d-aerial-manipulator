function [f_contact] = contact_dynamics(q,q_dot,...
                                        m_base, m_link, r, l,...
                                        wall)
%CONTACT_DYNAMICS function that computes the contact force at the EE

% Extract state and derivative
q = reshape(q, [7,1]);
q_dot = reshape(q_dot, [7,1]);

% Rotation Matrix
rot = @(theta) [cos(theta), -sin(theta);
                sin(theta), cos(theta)];

% Define the spring-damper-system that models the contact
k = 100;
d = 50;

% Find EE position and velocity
ee = q(1:2);
link = l.*[0;-1];
rotation = rot(q(3));

% Manipulator Links
for i =1:4
    rotation = rotation * rot(q(3 + i));
    ee = ee + rotation * link;
end

% EE Velocities
ee_dot = EE_Jacobian(q,l)*q_dot;

% Extract the planes normal vector
n = wall(1,:)';

% Next we compute the reaction force (2D-Force vector) at the EE. Only if 
% we are in contact. It is assumed that no other contact along the AM
% occures
dist = dot(n, ee - wall(2,:)');
f_contact = (dist <= 0) * (-k * dist ...
                         - (dot(n,ee_dot) <= 0) * d * dot(n,ee_dot))*n;

end

