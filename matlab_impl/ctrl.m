function u = ctrl(x,p,m_base,m_link, r,l)
%CTRL Chains the various controllers to compute the control action.
% x = Current System State (integral, value, derivative)
% p = Current Desired System State
% m_base = Mass of the base
% m_link = Mass of one individual link
% r = Rotor arm length
% l = Individual Link Length

% State dependent System inertia
q = x(8:14);
I = inertia(q, m_base,m_link,r,l);

u = [max(min(angle_lqri(x, I, r,...
                          pos_pd(x, p(1)) ...
                             )...
                + height_lqr(x, m_base + 4*m_link, p(2)),...
                [5;5]),...
               0);    % Bi-Rotor Inputs
               [0;5]... Tendon Inputs
            ];


end

