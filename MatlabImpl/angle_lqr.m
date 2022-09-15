function [u] = angle_lqr(x, m, r, t_des)
%LIN_LQR Summary of this function goes here
%   Detailed explanation goes here

% System Inertia
I = 0.83333 * m * (2 * r) ^ 2;

A_lqr = [0, 1;
         0, 0];

B_lqr = [zeros(1,2);
        1 / I * r, - 1 / I * r];

K = lqr(A_lqr, B_lqr, diag([100, 1]), 0.1 .* eye(2));

u = -K*([x(10);x(17)] - [t_des; 0]);
end

