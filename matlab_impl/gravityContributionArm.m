function [G] = gravityContributionArm(m_base, m_link)
%GRAVITYCONTRIBUTIONARM Function that returns the gravity contribution
%on the system 

% Gravity only accelerates our CoM downwards. There is no torque acting on
% the system due to gravity.
G_base = [0; 9.81 * (m_base + 4 * m_link);0];

% Similarly there are no effects on the arm joints due to gravity
G_mani = zeros(4,1);

% Putting it together
G = [G_base; G_mani];
end

