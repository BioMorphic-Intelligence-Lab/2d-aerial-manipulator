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

% Mass-Inertia Matrices
M = massInertiaMatrix(q,m_base,m_link,r,l);

% Corriolis Matrix
C = [zeros(3,3) ... Mass Inertia of the base is constant
     zeros(3,4); % Arm isn't -> effect on the base TODO 
    zeros(4,3),... No corriolis effect on the arm based on base velocity
    corriolisMatrixArm(q(4:7),q_dot(4:7),m_link,l)];

% Input Map Matrix
sT = sin(q(3));
cT = cos(q(3));
A = [-sT,-sT, zeros(1,2);
     cT,cT, zeros(1,2);
     r, - r, zeros(1,2);
    zeros(4,2), -r_tendon*ones(4,1),r_tendon*ones(4,1)];

% Damping
D = [diag([0.01, 0.01, 0.001]), zeros(3,4); % Damping linear to velocity for base
     zeros(4,3), diag([0.05,0.05,0.05,0.05])]; % Damping linear to joint velocity

% Joint Stiffness
K = [zeros(3,7);
     zeros(4,3), diag([0.5,0.5,0.5,0.5])] * q; % Linear in Joint Angle

% Gravity Compensation
G = gravityContributionArm(q,m_base,m_link,r,l);


end

