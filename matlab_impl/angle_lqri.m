function [u] = angle_lqri(x, m_base, m_link, r, l, t_des)
%LIN_LQR computes LQR optimal feedback to control the (base) system to a
%certain attitude
%   Neglects the arm and assumes the base to be a thin rod.

% Find current system inertia
I = inertia(x(8:14),m_base,m_link,r,l);

% Define system matrices
A_lqr = [0, 1, 0;
         0, 0, 1;
         0, 0, 0];

B_lqr = [zeros(2,2);
        1 / I * r, - 1 / I * r];

% Find optimal feedback gain matrix
K = lqr(A_lqr, B_lqr, diag([150, 500, 10]), 0.01 .* eye(2));

% Compute current disturbance torque
tau = tau_arm(x(8:14), m_link, l);
u_comp = -tau / (2* r) * [1;-1];

% LQRi control action + Disturbance torque Compensation
u = -K*([x(3);x(10);x(17)] - [0;t_des; 0]) + u_comp;
end

