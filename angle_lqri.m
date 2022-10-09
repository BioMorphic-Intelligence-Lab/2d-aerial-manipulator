function [u] = angle_lqri(x, m_base,m_link,l, r, t_des)
%LIN_LQR computes LQR optimal feedback to control the (base) system to a
%certain attitude

t_base = x(8);
t1 = x(9);
t2 = x(10);
t3 = x(11);
t4 = x(12);

% First find the Links CoMs positions relative to the drone CoM
link_coms = [l/2, 0, 0, 0;
             l, l/2, 0 ,0;
             l, l, l/2, 0;
             l, l, l, l/2] ...
        * [sin(t_base + t1), cos(t_base + t1);
          sin(t_base + t1 + t2), cos(t_base + t1 + t2);
          sin(t_base + t1 + t2 + t3), cos(t_base + t1 + t2 + t3);
          sin(t_base + t1 + t2 + t3 + t4), cos(t_base + t1 + t2 + t3 + t4)];

% Assume thin rod around its center, i.e I = 1/12 m L^2
I = 0.83333 * m_base * (2 * r) ^ 2 ...
    + m_link * (norm(link_coms(1,:))^2 ...
              + norm(link_coms(2,:))^2 ...
              + norm(link_coms(3,:))^2 ...
              + norm(link_coms(4,:))^2);

A_lqr = [0, 1, 0;
         0, 0, 1;
         0, 0, 0];

B_lqr = [zeros(2,2);
        1 / I * r, - 1 / I * r];

K = lqr(A_lqr, B_lqr, diag([10, 100, 10]), 0.1 .* eye(2));

% Control feedback
u = -K*[x(3);x(10)-t_des;x(17)];
end

