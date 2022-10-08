function [q, p] = static_equilibrium_link(t0,f,q_init,iter, m_base,m_link,l,r_tendon)
%STATIC_EQUILIBRIUm_link Function that com_linkputes the static equilibrium_link
%configuration q of the arm_link under gravity given the base orientation and
%tendon forces.
% Param_links
%   t0     := base orientation
%   f      := tendon forces
%   q_init := initial guess
%   iter   := num_link iterations
%   m_base := base m_linkass
%   m_link := link m_linkass
%   l      := link length
%   r_tendon:= distance of tendons to joints
% Returns
%   q := [t1;t2;t3;t4] arm_link configuration in equilibrium_link
%   p := [x;y] ee-position in equilibrium_link configuration
% 
% This function finds the equilibrium_link configuration by solving
% the equations of m_linkotion assum_linking q_ddot = q_dot = 0:
%       G(q,t0) + K * q = A * f
% for q. Because of G(q) this is not solvable in closed form_link.
% Hence we em_linkploy Newton's m_linkethod in configuration space to find the 
% roots of the function 
%       F(q) = G(q,t0) + K * q - A * f
% Using its Jacobian m_linkatrix 
%       J(q) = d/dq f(q) = d/dq G(q,t0) + K 
% the update step is
%   q_[n+1] = q - J(q_n)\F(q_n)

% Define Matrices that are constant throughout the iteration
g = 9.81;
A = [-r_tendon*ones(4,1),r_tendon*ones(4,1)];
K = diag([0.5,0.5,0.5,0.5]);

% Init newton variable
q = q_init;

for i=1:iter

    % Extract joint states
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);
    t4 = q(4);


    % Find current derivative of the gravity contribution
    dG = [1/2*g*l*m_link*(7*cos(t0 + t1) + 5*cos(t0 + t1 + t2) ...
           + 3*cos(t0 + t1 + t2 + t3) + cos(t0 + t1 + t2 + t3 + t4)),...
          1/2*g*l*m_link*(5*cos(t0 + t1 + t2) + 3*cos(t0 + t1 + t2 + t3) ...
           + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*(3*cos(t0 + t1 + t2 + t3) + cos(t0 + t1 + t2 + t3 + t4)),...
          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4);

          1/2*g*l*m_link*(5*cos(t0 + t1 + t2) + 3*cos(t0 + t1 + t2 + t3) ...
            + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*(5*cos(t0 + t1 + t2) + 3*cos(t0 + t1 + t2 + t3) ...
            + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*(3*cos(t0 + t1 + t2 + t3) + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4);

          1/2*g*l*m_link*(3*cos(t0 + t1 + t2 + t3) + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*(3*cos(t0 + t1 + t2 + t3) + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*(3*cos(t0 + t1 + t2 + t3) + cos(t0 + t1 + t2 + t3 + t4)), ...
          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4);

          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4), ...
          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4), ...
          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4), ...
          1/2*g*l*m_link*cos(t0 + t1 + t2 + t3 + t4)];
    
    % Find current gravity contribution
    G = gravityContributionArm([0;0;t0;q], m_base, m_link, l);

    % Find jacobian
    J = dG + K;
    
    % Find current function value, note we're only interested in the
    % gravity contributions to the robot joints
    F = G(4:end) + K*q - A*f;

    % Execute Newton's step
    q = q - J\F;
end

p = [0,0];


end



