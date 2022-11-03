function u = ctrl(x,p_des,m_base,m_link, r,l)
%CTRL Chains the various controllers to compute the control action.
% x = Current System State (integral, value, derivative)
% p_des = Current Desired Base Pose
% m_base = Mass of the base
% m_link = Mass of one individual link
% r = Rotor arm length
% l = Individual Link Length


% Desired base position
p_base = ee2base(p_des,x(11:14),l);

% Control Action
u = [max(min(angle_lqri(x, m_base, m_link, r, l,...
                          pos_pd(x, p_base(1)) ...
                             )...
                + height_lqr(x, m_base + 4*m_link, p_base(2)),...
                [5;5]),...
               0);    % Bi-Rotor Inputs
               [0;3]... Tendon Inputs
            ];


end

