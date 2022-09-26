function [angle] = pos_pd(x, m, des_pos)
%POS_LQR Function that returns a desired base attitude given a desired
%(lateral) position in a PD fashion

angle = (0.2 * (x(8) - des_pos) + 0.25 * x(15));

end

