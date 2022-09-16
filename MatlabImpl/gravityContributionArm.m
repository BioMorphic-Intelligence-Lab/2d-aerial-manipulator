function [G] = gravityContributionArm(q, m_link, l, t_base)
%GRAVITYCONTRIBUTIONARM Summary of this function goes here
%   Detailed explanation goes here

t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);

J1t = [ l*cos(t1)/2, 0,0,0;
       -l*sin(t1)/2, 0,0,0];
J2t = [ l*(cos(t1) + 1/2*cos(t1+t2)),  l/2*cos(t1+t2), 0,0;
       -l*(sin(t1) + 1/2*sin(t1+t2)), -l/2*sin(t1+t2), 0,0];
J3t = [l*(cos(t1) + cos(t1+t2) + 1/2*cos(t1+t2+t3)),...
       l*(cos(t1+t2)+1/2*cos(t1+t2+t3)),...
       l/2*cos(t1+t2+t3),0;
       -l*(sin(t1) + sin(t1+t2) + 1/2*sin(t1+t2+t3)),...
       -l*(sin(t1+t2)+1/2*sin(t1+t2+t3)),...
       -l/2*sin(t1+t2+t3),0];
J4t = [l*(cos(t1) + cos(t1+t2) + cos(t1+t2+t3) + 1/2*cos(t1+t2+t3+t4)),...
       l*(cos(t1+t2)+cos(t1+t2+t3) + 1/2*cos(t1+t2+t3+t4)),...
       l*(cos(t1+t2+t3)+1/2*cos(t1+t2+t3+t4)),...
       l/2*cos(t1+t2+t3+t4);
       -l*(sin(t1) + sin(t1+t2) + sin(t1+t2+t3) + 1/2*sin(t1+t2+t3+t4)),...
       -l*(sin(t1+t2)+sin(t1+t2+t3)+1/2*sin(t1+t2+t3+t4)),...
       -l*(sin(t1+t2+t3) + 1/2*sin(t1+t2+t3+t4)),...
       -l/2*sin(t1+t2+t3+t4)];

G_mani_1 = 9.81 * m_link * J1t'*[0;-1];
G_mani_2 = 9.81 * m_link * J2t'*[0;-1];
G_mani_3 = 9.81 * m_link * J3t'*[0;-1];
G_mani_4 = 9.81 * m_link * J4t'*[0;-1];;
% Putting it together
G = G_mani_1 + G_mani_2 + G_mani_3 + G_mani_4;

end

