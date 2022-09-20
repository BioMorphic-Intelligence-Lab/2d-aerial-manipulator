function [f] = estimate_force(q, q_dot, q_ddot, u,...
                              m_base, m_link, r, l, r_tendon)
%ESTIMATE_FORCE Summary of this function goes here
%   Detailed explanation goes here
q = reshape(q, [7,1]);
q_dot = reshape(q_dot, [7,1]);
q_ddot = reshape(q_ddot, [7,1]);
u = reshape(u,[4,1]);


J_ee = EE_Jacobian(q,l);
[M, C, A, D, K, G] = eom_matrices(q,q_dot,m_base,m_link,r,l,r_tendon); 

f = pinv(J_ee')*(M*q_ddot + (C + D)*q_dot + G + K - A*u);

end

