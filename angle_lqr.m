function [u] = angle_lqr(x, m, r, t_des)
%LIN_LQR computes LQR optimal feedback to control the (base) system to a
%certain attitude
%   Neglects the arm and assumes the base to be a thin rod.

% System Inertia
I = 0.83333 * m * (2 * r) ^ 2;

A_lqr = [0, 1;
         0, 0];

B_lqr = [zeros(1,2);
        1 / I * r, - 1 / I * r];

K = lqr(A_lqr, B_lqr, diag([100, 1]), 0.1 .* eye(2));

% Control feedback
u = -K*([x(10);x(17)] - [t_des; 0]);
end

