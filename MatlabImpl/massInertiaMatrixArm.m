function M = massInertiaMatrixArm(q,m,l)
%MASSINERTIAMATRIXARM Summary of this function goes here
%   Detailed explanation goes here

t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);

% First Row
M11 = (l^2*m*(25 + 15*cos(t2) + 9*cos(t3) + 9*cos(t2 + t3) ...
      + 3*cos(t4) + 3*cos(t3 + t4) + 3*cos(t2 + t3 + t4)))/3;
M12 = (l^2*m*(19 + 10*cos(t2) + 12*cos(t3) + 6*cos(t2 + t3) + 4*cos(t4)...
      + 4*cos(t3 + t4) + 2*cos(t2 + t3 + t4)))/4;
M13 = (l^2*m*(13 + 9*cos(t3) + 9*cos(t2 + t3) + 6*cos(t4) ...
       + 3*cos(t3 + t4) + 3*cos(t2 + t3 + t4)))/6;
M14 = (l^2*m*(7 + 6*cos(t4) + 6*cos(t3 + t4) + 6*cos(t2 + t3 + t4)))/12;

% Second Row
M21 = (l^2*m*(19 + 10*cos(t2) + 12*cos(t3) + 6*cos(t2 + t3) + 4*cos(t4) ...
       + 4*cos(t3 + t4) + 2*cos(t2 + t3 + t4)))/4;
M22 = (l^2*m*(19 + 12*cos(t3) + 4*cos(t4) + 4*cos(t3 + t4)))/4;
M23 = (l^2*m*(13 + 9*cos(t3) + 6*cos(t4) + 3*cos(t3 + t4)))/6;
M24 = (l^2*m*(7 + 6*cos(t4) + 6*cos(t3 + t4)))/12;

% Thirst Row
M31 = (l^2*m*(13 + 9*cos(t3) + 9*cos(t2 + t3) + 6*cos(t4) ...
       + 3*cos(t3 + t4) + 3*cos(t2 + t3 + t4)))/6;
M32 = (l^2*m*(13 + 9*cos(t3) + 6*cos(t4) + 3*cos(t3 + t4)))/6;
M33 = (l^2*m*(13 + 6*cos(t4)))/6;
M34 = (l^2*m*(7 + 6*cos(t4)))/12;

% Fourth Row
M41 = (l^2*m*(7 + 6*cos(t4) + 6*cos(t3 + t4) + 6*cos(t2 + t3 + t4)))/12;
M42 = (l^2*m*(7 + 6*cos(t4) + 6*cos(t3 + t4)))/12;
M43 = (l^2*m*(7 + 6*cos(t4)))/12;
M44 = (7*l^2*m)/12;

% Collecting it in one matrix
M = [M11, M12, M13,M14;
     M21, M22, M23,M24;
     M31, M32, M33,M34;
     M41, M42, M43,M44];
end

