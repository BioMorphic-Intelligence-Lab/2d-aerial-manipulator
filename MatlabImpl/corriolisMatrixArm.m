function C = corriolisMatrixArm(q,q_dot,m,l)
%MASsinERTIAMATRIXARM Summary of this function goes here
%   Detailed explanation goes here

t1 = q(1);
t2 = q(2);
t3 = q(3);
t4 = q(4);

t1d = q_dot(1);
t2d = q_dot(2);
t3d = q_dot(3);
t4d = q_dot(4);

% First Row
C11 = -l^2*m*((5 * sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4))*t2d...
       +(3*sin(t3)+3*sin(t2+t3)+sin(t3+t4)+sin(t2+t3+t4))*t3d...
       +(sin(t4)+sin(t3+t4)+sin(t2+t3+t4)) * t4d);
C12 = -(1/2)*l^2*m*((5*sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4))*t2d...
       +(6*sin(t3)+3*sin(t2+t3)+2*sin(t3+t4)+sin(t2+t3+t4))*t3d...
       +(2*sin(t4)+2*sin(t3+t4)+sin(t2+t3+t4))*t4d);
C13 = -(1/2)*l^2*m*((3*sin(t2+t3)+sin(t2+t3+t4))*t2d...
      +(3*sin(t3)+3*sin(t2+t3)+sin(t3+t4)+sin(t2+t3+t4))*t3d...
      +(2*sin(t4)+sin(t3+t4)+sin(t2+t3+t4))*t4d);
C14 = -(1/2)*l^2*m*(sin(t2+t3+t4)*t2d...
       +(sin(t3+t4)+sin(t2+t3+t4))*t3d...
       +(sin(t4)+sin(t3+t4)+sin(t2+t3+t4))*t4d);

% Second Row
C21 = 1/4*l^2*m*(2*(5*sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4))*t1d...
      -(5*sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4))*t2d-12*sin(t3)*t3d...
      -3*sin(t2+t3)*t3d-4*sin(t3+t4)*t3d-sin(t2+t3+t4)*t3d...
      -4*sin(t4)*t4d-4*sin(t3+t4)*t4d-sin(t2+t3+t4)*t4d);
C22 = 1/4*l^2*m*((5*sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4))*t1d...
      -4*((3*sin(t3)+sin(t3+t4))*t3d+(sin(t4)+sin(t3+t4))*t4d));
C23 = 1/4*l^2*m*((3*sin(t2+t3)+sin(t2+t3+t4))*t1d...
      -2*((3*sin(t3)+sin(t3+t4))*t3d+(2*sin(t4)+sin(t3+t4))*t4d));
C24 = 1/4*l^2*m*(sin(t2+t3+t4)*t1d-2*(sin(t3+t4)*t3d...
      +(sin(t4)+sin(t3+t4))*t4d));

% Thirst Row
C31 = 1/4*l^2*m*(2*(3*sin(t3)+3*sin(t2+t3)+sin(t3+t4)...
      +sin(t2+t3+t4))*t1d-(-6*sin(t3)+3*sin(t2+t3)-2*sin(t3+t4)...
      +sin(t2+t3+t4))*t2d-3*sin(t3)*t3d-3*sin(t2+t3)*t3d-sin(t3+t4)*t3d...
      -sin(t2+t3+t4)*t3d-4*sin(t4)*t4d-sin(t3+t4)*t4d-sin(t2+t3+t4)*t4d);
C32 = 1/4*l^2*m*((6*sin(t3)+3*sin(t2+t3)+2*sin(t3+t4)...
      +sin(t2+t3+t4))*t1d+2*(3*sin(t3)+sin(t3+t4))*t2d-3*sin(t3)*t3d...
      -sin(t3+t4)*t3d-4*sin(t4)*t4d-sin(t3+t4)*t4d);
C33 = 1/4*l^2*m*((3*sin(t3)+3*sin(t2+t3)+sin(t3+t4)+sin(t2+t3+t4))*t1d...
      +(3*sin(t3)+sin(t3+t4))*t2d-4*sin(t4)*t4d);
C34 = 1/4*l^2*m*((sin(t3+t4)+sin(t2+t3+t4))*t1d...
      +sin(t3+t4)*t2d-2*sin(t4)*t4d);

% Fourth Row
C41 = 1/4*l^2*m*(2*(sin(t4)+sin(t3+t4)+sin(t2+t3+t4))*t1d...
      +(2*sin(t4)+2*sin(t3+t4)-sin(t2+t3+t4))*t2d+2*sin(t4)*t3d...
      -sin(t3+t4)*t3d-sin(t2+t3+t4)*t3d-sin(t4)*t4d-sin(t3+t4)*t4d...
      -sin(t2+t3+t4)*t4d);
C42 = 1/4*l^2*m*((2*sin(t4)+2*sin(t3+t4)+sin(t2+t3+t4))*t1d+2*(sin(t4)...
      +sin(t3+t4))*t2d+2*sin(t4)*t3d-sin(t3+t4)*t3d-sin(t4)*t4d...
      -sin(t3+t4)*t4d);
C43 = 1/4*l^2*m*((2*sin(t4)+sin(t3+t4)+sin(t2+t3+t4))*t1d...
      +(2*sin(t4)+sin(t3+t4))*t2d+sin(t4)*(2*t3d-t4d));
C44 = 1/4*l^2*m*((sin(t4)+sin(t3+t4)+sin(t2+t3+t4))*t1d...
      +(sin(t4)+sin(t3+t4))*t2d+sin(t4)*t3d);

% Collecting it in one matrix
C = [C11, C12, C13,C14;
     C21, C22, C23,C24;
     C31, C32, C33,C34;
     C41, C42, C43,C44];
end

