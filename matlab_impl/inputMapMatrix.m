function [A] = inputMapMatrix(q,r,l, r_tendon)
%INPUTMAPMATRIX Function that computes the map between control inputs and
%generalized forces

% Extract System State
t_base = q(3);
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);

% Effect on the system state directly due to actuation
sT = sin(t_base);
cT = cos(t_base);
A_direct = [-sT,-sT, zeros(1,2);
             cT,cT, zeros(1,2);
             r, - r, zeros(1,2);
             zeros(4,2), -r_tendon*ones(4,1),r_tendon*ones(4,1)];

% Effect on the system due to the counter force to the actuation. This is
% similar to a gravity contribution in a convetional fixed base manipulator
% except for that the force action on the links isn't gravity but it's the
% counter force to the one produce by the propellers.

% First the effect on the base
coms = linkCoMs(q,l);
tau_base = -[0,0,0,0;
            0,0,0,0;
            1,1,0,0;
            zeros(4,4)] ...
   *(norm(coms(1,:))*sin(t1)...
   + norm(coms(2,:))*sin(t1 + t2)...
   + norm(coms(3,:))*sin(t1 + t2 + t3)...
   + norm(coms(4,:))*sin(t1 + t2 + t3 + t4));

% Then the effect on the arm. Fist we find the Jacobians
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

% Find the matrix that relatex rotor force to resulting joint torques
R = [cT,-sT;
     sT, cT];
tau_arm = [0,0,0,0;
           0,0,0,0;
           0,0,0,0;
           (J1t' + J2t' + J3t' + J4t')*R, zeros(4,2)];

% Putting it all in one matrix

A = A_direct + tau_base;% + tau_arm;

end

