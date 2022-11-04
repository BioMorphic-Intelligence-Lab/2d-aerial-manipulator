function [M, C, A, D, K, G] = eom_matrices(q,q_dot,m_base,m_link,r,l,r_tendon)
%EOM_MATRICES function that returns the Equaton of Motion Matrices given
%the current state
% Input:
%     q current state
%     q_dot current velocities
%     m = m_base + m_link system mass
%     r rotor arm length
%     l robot arm link length
%     r_tendon distance of tendon to backbone
% Output:
%     M Mass/Inertia matrix (Including dynamic coupling)
%     C Coriolis Matrix
%     A Input to generalized forces matrix
%     D System Damping
%     K System Stiffness
%     G Gravity contribution

% Mass-Inertia Matrices
M = massInertiaMatrix(q,m_base,m_link,r,l);

% Corriolis Matrix
C = coriolisMatrix(q,q_dot,m_link,l);

% Input Map Matrix
A = inputMapMatrix(q,r,l,r_tendon);

% Damping
D = [diag([0.01, 0.01, 0.001]), zeros(3,4); % Damping linear to velocity for base
     zeros(4,3), diag([0.05,0.05,0.05,0.05])]; % Damping linear to joint velocity

% Joint Stiffness
K = [zeros(3,7);
     zeros(4,3), diag([0.5,0.5,0.5,0.5])] * q; % Linear in Joint Angle

% Gravity Compensation
G = gravityContributionArm(m_base,m_link);


end

