function [M, A, G] = eom_matrices(q, m, r)
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
I = 0.83333 * m * (2 * r) ^ 2;
% Mass-Inertia Matrix
M = [m, 0, 0;
     0, m, 0;
     0, 0, I];

% Input Map Matrix
sT = sin(q(3));
cT = cos(q(3));
A = [-sT,-sT;
     cT,cT;
     r, - r];

% Gravity Compensation
G = [0; 9.81 * m; 0];


end

