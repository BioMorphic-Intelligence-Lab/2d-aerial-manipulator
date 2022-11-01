function [G] = gravityContributionArm(q, m_base, m_link, r, l)
%GRAVITYCONTRIBUTIONARM Function that returns the gravity contribution
%on the system 

% Extract System State
t_base = q(3);
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);


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

G_mani_1 = 9.81 * m_link * J1t'*[0;-1];
G_mani_2 = 9.81 * m_link * J2t'*[0;-1];
G_mani_3 = 9.81 * m_link * J3t'*[0;-1];
G_mani_4 = 9.81 * m_link * J4t'*[0;-1];

% Putting it together
G_mani = G_mani_1 + G_mani_2 + G_mani_3 + G_mani_4;

% First find the Links CoMs positions relative to the drone CoM
link_coms = [l/2, 0, 0, 0;
             l, l/2, 0 ,0;
             l, l, l/2, 0;
             l, l, l, l/2] ...
        * [sin(t_base + t1), cos(t_base + t1);
          sin(t_base + t1 + t2), cos(t_base + t1 + t2);
          sin(t_base + t1 + t2 + t3), cos(t_base + t1 + t2 + t3);
          sin(t_base + t1 + t2 + t3 + t4), cos(t_base + t1 + t2 + t3 + t4)];

% Find the gravity effect on the base
tau_base = m_link*9.81...
    *(norm(link_coms(1,:))*sin(t_base + t1)...
   + norm(link_coms(2,:))*sin(t_base + t1 + t2)...
   + norm(link_coms(3,:))*sin(t_base + t1 + t2 + t3)...
   + norm(link_coms(4,:))*sin(t_base + t1 + t2 + t3 + t4));
G_base = [0; 9.81 * (m_base + 4 * m_link); tau_base];

% Putting it together
G = [G_base; G_mani];
end

