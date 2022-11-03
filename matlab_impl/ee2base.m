function [p] = ee2base(p_ee, q, l)
%EE2BASE Function that computes the required base position to achive a
%certain EE position given the current arm configuration in steady state,
%i.e. base attitude = 0
% p_ee = Desired EE position
% q    = current robot arm configuration
% l    = robot arm link length

t = [0;0];
R = eye(2);
for i=1:length(q)
    R = R * [cos(q(i)), -sin(q(i));
             sin(q(i)), cos(q(i))];
    t = t + R * [0;-l]; 
end

% Required Base pose. Zero orientation due to steady state.
p = [p_ee - t;0];

end

