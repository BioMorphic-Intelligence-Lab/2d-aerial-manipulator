function [f_contact] = contact_dynamics(q,q_dot,...
                                        m_base, m_link, r, l,...
                                        x_wall)
%CONTACT_DYNAMICS Summary of this function goes here

% Rotation Matrix
rot = @(theta) [cos(theta), -sin(theta);
                sin(theta), cos(theta)];

% Define the spring-damper-system that models the contact
k = 10;
d = 5;

% Find EE position and velocity
ee = q(1:2);
link = l.*[0;-1];
rotation = rot(q(3));

% Manipulator Links
for k =1:4
    rotation = rotation * rot(q(3 + k));
    ee = ee + rotation * link;
end


ee_dot = [0;0];


% Next we compute the reaction force (2D-Force vector) at the EE. Only if 
% we are in contact. It is assumed that no other contact along the AM
% occures
f_contact = [(ee(1) >= x_wall) * (k * (x_wall - ee(1)) - d * ee_dot(1)); 0];

end

