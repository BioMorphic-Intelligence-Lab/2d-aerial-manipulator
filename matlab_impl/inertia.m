function [I] = inertia(q, m_base, m_link, r, l)
%I Function that computes the state dependent Inertia of the AM
%   Detailed explanation goes here

% First find the Links CoMs positions relative to the drone CoM
link_coms = linkCoMs(q, l);

% Assume thin rod around its center, i.e I = 1/12 m L^2
% And links assumed to be point masses at their CoM
I = 0.83333 * m_base * (2 * r) ^ 2 ...
    + m_link * (norm(link_coms(1,:))^2 ...
              + norm(link_coms(2,:))^2 ...
              + norm(link_coms(3,:))^2 ...
              + norm(link_coms(4,:))^2);


end

