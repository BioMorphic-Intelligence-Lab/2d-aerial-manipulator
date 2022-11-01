function [u] = height_lqr(x, m, des_height)
%HEIGHT_LQRi Function that computes LQRi optimal feedback control to regulate
%the altitude of the system

A_lqr = [0, 1;
         0, 0];

B_lqr = [zeros(1,2);
        1 / m, 1 / m];

K = lqr(A_lqr, B_lqr, diag([100, 40]), 0.1 .* eye(2));

% LQR control action + Gravity Compensation
u = -K*[x(9)-des_height;x(16)] + m * 9.81 / cos(x(10)) .* [0.5; 0.5];
end

