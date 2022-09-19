function [M] = massInertiaMatrix(q, m_base, m_link,r,l)
%MASSINERTIAMATRIX Summary of this function goes here
%   Detailed explanation goes here

% Extract states
t_base = q(3);
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);

% First find the Links CoMs positions relative to the drone CoM
link_coms = [l/2, 0, 0, 0;
             l, l/2, 0 ,0;
             l, l, l/2, 0;
             l, l, l, l/2] ...
        * [sin(t_base + t1), cos(t_base + t1);
          sin(t_base + t1 + t2), cos(t_base + t1 + t2);
          sin(t_base + t1 + t2 + t3), cos(t_base + t1 + t2 + t3);
          sin(t_base + t1 + t2 + t3 + t4), cos(t_base + t1 + t2 + t3 + t4)];

% Assume thin rod around its center, i.e I = 1/12 m L^2
I = 0.83333 * m_base * (2 * r) ^ 2 ...
    + norm(link_coms(1,:)*m_link)...
    + norm(link_coms(2,:)*m_link)...
    + norm(link_coms(3,:)*m_link)...
    + norm(link_coms(4,:)*m_link);

% Mass-Inertia Matrix Base
M_base = [m_base, 0, 0;
         0, m_base, 0;
         0, 0, I];

% Arm Inertia matrix
% First Row
M11 = (l^2*m_link*(25 + 15*cos(t2) + 9*cos(t3) + 9*cos(t2 + t3) ...
      + 3*cos(t4) + 3*cos(t3 + t4) + 3*cos(t2 + t3 + t4)))/3;
M12 = (l^2*m_link*(19 + 10*cos(t2) + 12*cos(t3) + 6*cos(t2 + t3) + 4*cos(t4)...
      + 4*cos(t3 + t4) + 2*cos(t2 + t3 + t4)))/4;
M13 = (l^2*m_link*(13 + 9*cos(t3) + 9*cos(t2 + t3) + 6*cos(t4) ...
       + 3*cos(t3 + t4) + 3*cos(t2 + t3 + t4)))/6;
M14 = (l^2*m_link*(7 + 6*cos(t4) + 6*cos(t3 + t4) + 6*cos(t2 + t3 + t4)))/12;

% Second Row
M21 = (l^2*m_link*(19 + 10*cos(t2) + 12*cos(t3) + 6*cos(t2 + t3) + 4*cos(t4) ...
       + 4*cos(t3 + t4) + 2*cos(t2 + t3 + t4)))/4;
M22 = (l^2*m_link*(19 + 12*cos(t3) + 4*cos(t4) + 4*cos(t3 + t4)))/4;
M23 = (l^2*m_link*(13 + 9*cos(t3) + 6*cos(t4) + 3*cos(t3 + t4)))/6;
M24 = (l^2*m_link*(7 + 6*cos(t4) + 6*cos(t3 + t4)))/12;

% Thirst Row
M31 = (l^2*m_link*(13 + 9*cos(t3) + 9*cos(t2 + t3) + 6*cos(t4) ...
       + 3*cos(t3 + t4) + 3*cos(t2 + t3 + t4)))/6;
M32 = (l^2*m_link*(13 + 9*cos(t3) + 6*cos(t4) + 3*cos(t3 + t4)))/6;
M33 = (l^2*m_link*(13 + 6*cos(t4)))/6;
M34 = (l^2*m_link*(7 + 6*cos(t4)))/12;

% Fourth Row
M41 = (l^2*m_link*(7 + 6*cos(t4) + 6*cos(t3 + t4) + 6*cos(t2 + t3 + t4)))/12;
M42 = (l^2*m_link*(7 + 6*cos(t4) + 6*cos(t3 + t4)))/12;
M43 = (l^2*m_link*(7 + 6*cos(t4)))/12;
M44 = (7*l^2*m_link)/12;

% Collecting it in one matrix
M_arm = [M11, M12, M13,M14;
     M21, M22, M23,M24;
     M31, M32, M33,M34;
     M41, M42, M43,M44];


% Coupling Matrix
H = [
    1/2*l*m_link*(7*cos(t1)+5*cos(t1+t2)+3*cos(t1+t2+t3)+cos(t1+t2+t3+t4)),...
    1/2*l*m_link*(5*cos(t1+t2)+3*cos(t1+t2+t3)+cos(t1+t2+t3+t4)),...
    1/2*l*m_link*(3*cos(t1+t2+t3)+cos(t1+t2+t3+t4)),...
    1/2*l*m_link*cos(t1+t2+t3+t4); % Coupling effect for x-axis

    -(1/2)*l*m_link*(7*sin(t1)+5*sin(t1+t2)+3*sin(t1+t2+t3)+sin(t1+t2+t3+t4)),...
    -(1/2)*l*m_link*(5*sin(t1+t2)+3*sin(t1+t2+t3)+sin(t1+t2+t3+t4)),...
    -(1/2)*l*m_link*(3*sin(t1+t2+t3)+sin(t1+t2+t3+t4)),...
    -(1/2)*l*m_link*sin(t1+t2+t3+t4); % Coupling effect for y-axis

    (4*l^2*m_link)/3,...
    -(1/2)*l^2*m_link*(-2+5*sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4)),...
    -(1/6)*l^2*m_link*(-4+9*sin(t3)+9*sin(t2+t3)+3*sin(t3+t4)+3*sin(t2+t3+t4)),...
    -(1/6)*l^2*m_link*(-2+3*sin(t4)+3*sin(t3+t4)+3*sin(t2+t3+t4));
    ]; % Coupling effect for rotation

% Putting it all together

M = [M_base, H;
     H', M_arm];

end
