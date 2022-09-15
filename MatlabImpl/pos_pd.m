function [angle] = pos_pd(x, m, des_pos)
%POS_LQR Summary of this function goes here
%   Detailed explanation goes here

angle = (0.2 * (x(8) - des_pos) + 0.25 * x(15));

end

