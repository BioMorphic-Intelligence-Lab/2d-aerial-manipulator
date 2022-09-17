function [M, C, A, D, K, G] = eom_matrices(q,q_dot,m_base,m_link,r,l,r_tendon)
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

% Corriolis Matrix
C = [zeros(3,7); % No corriolis effect on the base
    zeros(4,3),... No corriolis effect on the arm based on base velocity
    corriolisMatrixArm(q(4:7),q_dot(4:7),m_link,l)];

% Input Map Matrix
sT = sin(q(3));
cT = cos(q(3));
A = [-sT,-sT, zeros(1,2);
     cT,cT, zeros(1,2);
     r, - r, zeros(1,2);
     % TODO Double check this. The arm should reach equilibrium but
     % oscillates... Maybe the damping/stiffness is not correct? Or the input map.
     zeros(4,2), -r_tendon*ones(4,1),r_tendon*ones(4,1)];

% Damping
D = [diag([0.01, 0.01, 0.001]), zeros(3,4); % Damping linear to velocity for base
     zeros(4,3), diag([0.05,0.05,0.05,0.05])]; % Damping linear to joint velocity

% Joint Stiffness
K = [zeros(3,7);
    zeros(4,3), diag([0.01,0.01,0.01,0.01])] * q; % Linear in Joint Angle

% Gravity Compensation
G = [0; 9.81 * m_base; 0; gravityContributionArm(q(4:7),m_link,l,q(3))];


end

