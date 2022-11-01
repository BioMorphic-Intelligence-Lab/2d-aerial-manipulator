function [tau] = tau_arm(q, m_link, l)
%TAU_ARM Function that computes the disturbance torque induced on the base,
%from the current configuration of the arm (i.e. the static disturbance)

% First find the current CoMs
linkcoms = linkCoMs(q, l);

% Torque enacted on the base is r_com x F = m (r_com x g) = m (r_com,x * 9.81) 
tau = sum(linkcoms(:,1)*m_link*9.81); 

end

