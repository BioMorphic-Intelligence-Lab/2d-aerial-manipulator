function [u] = height_lqri(x, m, r, des_height)
%LIN_LQR Summary of this function goes here
%   Detailed explanation goes here


A_lqr = [0, 1, 0;
         0, 0, 1;
         0, 0, 0];

B_lqr = [zeros(2,2);
        1 / m, 1 / m];

K = lqr(A_lqr, B_lqr, diag([25, 100, 20]), 0.1 .* eye(2));


u = -K*[x(2);x(9)-des_height;x(16)] + m * 9.81 / cos(x(10)) .* [0.5; 0.5];
end

