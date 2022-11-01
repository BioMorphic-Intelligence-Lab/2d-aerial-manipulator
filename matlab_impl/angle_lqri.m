function [u] = angle_lqri(x, I, r, t_des)
%LIN_LQR computes LQR optimal feedback to control the (base) system to a
%certain attitude
%   Neglects the arm and assumes the base to be a thin rod.

A_lqr = [0, 1, 0;
         0, 0, 1;
         0, 0, 0];

B_lqr = [zeros(2,2);
        1 / I * r, - 1 / I * r];

K = lqr(A_lqr, B_lqr, diag([150, 500, 10]), 0.01 .* eye(2));

% Control feedback
u = -K*([x(3);x(10);x(17)] - [0;t_des; 0]);
end

