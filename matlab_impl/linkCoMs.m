function [link_coms] = linkCoMs(q, l)
%LINKCOMS Function that finds the coordinates of the link CoMs relative to
%the base CoM

t_base = q(3);
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);

link_coms = [l/2, 0, 0, 0;
             l, l/2, 0 ,0;
             l, l, l/2, 0;
             l, l, l, l/2] ...
        * [sin(t_base + t1), cos(t_base + t1);
          sin(t_base + t1 + t2), cos(t_base + t1 + t2);
          sin(t_base + t1 + t2 + t3), cos(t_base + t1 + t2 + t3);
          sin(t_base + t1 + t2 + t3 + t4), cos(t_base + t1 + t2 + t3 + t4)];
end

