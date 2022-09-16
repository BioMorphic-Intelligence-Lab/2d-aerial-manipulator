function [M, A, G] = eom_matrices(q, m_base, m_link, r,l)
%EOM_MATRICES function that returns the Equaton of Motion Matrices given
%the current state
% Input:
%     q current state
%     m base mass
%     r arm length
% Output:
%     M Mass/Inertia matrix
%     A Input to generalized forces matrix
%     G Gravity contribution

% Compute Inertia around the the CoM
% Assume thin rod around its center, i.e I = 1/12 m L^2
I = 0.83333 * m_base * (2 * r) ^ 2;
% Mass-Inertia Matrices
M_base = [m_base, 0, 0;
         0, m_base, 0;
         0, 0, I];
M_arm = massInertiaMatrixArm(q(4:7), m_link, l);

H = zeros(3,4); % Coupling Matrix  

% Putting it all together
M = [M_base, H;
     H', M_arm];


% Input Map Matrix
sT = sin(q(3));
cT = cos(q(3));
A = [-sT,-sT;
     cT,cT;
     r, - r;
     zeros(4,2)];

% Gravity Compensation
G = [0; 9.81 * m_base; 0; gravityContributionArm(q(4:7),m_link,l,q(3))];


end

