function [f_virtual] = virtual_force(q,u,m_base,m_link,r,l)
%VIRTUAL_FORCE Function that computes the virtual force arising from having
%a non-inertial reference frame (the CoM of the Base)

% Our frame of reference (CoM of the Base) is moving
% hence we have virtual forces arising. It's always of the same magnitude,
% related to the frame acceleration
% Extract System State
t_base = q(3);
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);

% First we must compute the current reference frame acceleration
a = [-sin(t_base) * (u(1) + u(2));
     cos(t_base)*(u(1) + u(2)) - 9.81 * (m_base + 4*m_link)];

% Initialize the virtual force vector. There is no virtual force appearing
% in the linear positions
f_virtual = zeros(7,1);

% Find the link CoMs with respect to the base
coms = linkCoMs(q,l);

% Virtual Contribution to the base orientation
f_virtual(3) = norm(a) * m_link...
    *(norm(coms(1,:))...
   + norm(coms(2,:))...
   + norm(coms(3,:))...
   + norm(coms(4,:)));

% Define Joint Jacobians
% Gravity Effect on the arm
J1t = [ l*cos(t_base+t1)/2, 0,0,0;
       -l*sin(t_base+t1)/2, 0,0,0];
J2t = [ l*(cos(t_base+t1) + 1/2*cos(t_base+t1+t2)),  l/2*cos(t_base+t1+t2), 0,0;
       -l*(sin(t_base+t1) + 1/2*sin(t_base+t1+t2)), -l/2*sin(t_base+t1+t2), 0,0];
J3t = [l*(cos(t_base+t1) + cos(t_base+t1+t2) + 1/2*cos(t_base+t1+t2+t3)),...
       l*(cos(t_base+t1+t2)+1/2*cos(t_base+t1+t2+t3)),...
       l/2*cos(t_base+t1+t2+t3),0;
       -l*(sin(t_base+t1) + sin(t_base+t1+t2) + 1/2*sin(t_base+t1+t2+t3)),...
       -l*(sin(t_base+t1+t2)+1/2*sin(t_base+t1+t2+t3)),...
       -l/2*sin(t_base+t1+t2+t3),0];
J4t = [l*(cos(t_base+t1) + cos(t_base+t1+t2) + cos(t_base+t1+t2+t3) + 1/2*cos(t_base+t1+t2+t3+t4)),...
       l*(cos(t_base+t1+t2)+cos(t_base+t1+t2+t3) + 1/2*cos(t_base+t1+t2+t3+t4)),...
       l*(cos(t_base+t1+t2+t3)+1/2*cos(t_base+t1+t2+t3+t4)),...
       l/2*cos(t_base+t1+t2+t3+t4);
       -l*(sin(t_base+t1) + sin(t_base+t1+t2) + sin(t_base+t1+t2+t3) + 1/2*sin(t_base+t1+t2+t3+t4)),...
       -l*(sin(t_base+t1+t2)+sin(t_base+t1+t2+t3)+1/2*sin(t_base+t1+t2+t3+t4)),...
       -l*(sin(t_base+t1+t2+t3) + 1/2*sin(t_base+t1+t2+t3+t4)),...
       -l/2*sin(t_base+t1+t2+t3+t4)];

% Virtual Contribution to the Joints
f_virtual(4:end) = m_link * (J1t' + J2t' + J3t' + J4t') * a;

end