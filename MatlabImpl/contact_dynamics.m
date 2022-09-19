function [x_dot_contact] = contact_dynamics(x,x_dot,...
                                            m_base, m_link, r, l,r_tendon,...
                                            contact_x)
%CONTACT_DYNAMICS Summary of this function goes here

% The resulting derivative is the previous derivative + the reaction 
% force at from the contact. Hence initially we copy the derivative
x_dot_contact = x_dot;

% Next we compute the reaction force (2D-Force vector) at the EE. Only if 
% we are in contact. It is assumed that no other contact along the AM
% occures
f_reaction = (x(1) >= contact_x) * 0;
end

