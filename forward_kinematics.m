function [x,y] = forward_kinematics(q,l)
%FORWARD_KINEMATICS Computes the EE position given the joint configuration

p = [q(1);q(2)];
for i=1:4
    angle = sum(q(3:3+i));
    rot = [cos(angle),-sin(angle);
           sin(angle), cos(angle)];
    t = rot*[0; -l];
    p = p + t;
end

x = p(1);
y = p(2);

end



