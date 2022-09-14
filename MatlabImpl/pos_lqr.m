function [angle] = pos_lqr(x, m, des_pos)
%POS_LQR Summary of this function goes here
%   Detailed explanation goes here

angle = (0.2 * (x(4) - des_pos) + 0.25 * x(7));

end

