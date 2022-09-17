function am = am(x,u, m_base, m_link, r, l,r_tendon, q_des)
%AM Computes the state derivative given the current state andc nput
%   The state consists of the dualrotor pose and velocities
%   x is the current state
%   u is the current control input
%   m system weight
%   r arm length
%   q_des desired state

% Extract pose and velocities
q_base = x(8:10);
q_mani = x(11:14);
q_dot_base = x(15:17);
q_dot_mani = x(18:21);
 
% Initialize the derivative vector
x_dot = zeros(21,1);

% The rate of change of the integral is just the value (difference)
x_dot(1:7) = [q_base; q_mani] - q_des;
% It's a first order system so the pose derivative is just the velocities
x_dot(8:14) = [q_dot_base; q_dot_mani];

% Define the equation of motion matrices
[M, C, A, D, K, G] = eom_matrices([q_base;q_mani], [q_dot_base;q_dot_mani],...
                            m_base, m_link, r, l,r_tendon);
% The velocities derivatives (accelerations) are computed via the equations
% of motion of the dualrotor
x_dot(15:21) = M\(A * u - C*[q_dot_base;q_dot_mani] ...
                  - D*[q_base; q_mani] - K - 0);

% Store the derivative in the return value
am = x_dot;
end

