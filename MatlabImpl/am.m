function am = am(x,u, m, r, p_des)
%AM Computes the state derivative given the current state andc nput
%   The state consists of the dualrotor pose and velocities
%   x is the current state
%   u is the current control input
%   m system weight
%   r arm length
%   p_des desired pose

% Extract pose and velocities
q = x(4:6);
q_dot = x(7:9);

% Initialize the derivative vector
x_dot = zeros(9,1);

% The rate of change of the integral is just the value (difference)
x_dot(1:3) = q - p_des;
% It's a first order system so the pose derivative is just the velocities
x_dot(4:6) = q_dot;

% Define the equation of motion matrices
[M, A, G] = eom_matrices(q, m, r);
% The velocities derivatives (accelerations) are computed via the equations
% of motion of the dualrotor
x_dot(7:9) = M\(A * u - G);

% Store the derivative in the return value
am = x_dot;
end

