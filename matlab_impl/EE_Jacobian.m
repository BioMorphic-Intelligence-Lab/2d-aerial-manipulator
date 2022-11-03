function [Jee] = EE_Jacobian(q,l)
%EE_JACOBIAN Function that returns the state dependent EE Jacobian

% Extract state
tB = q(3);
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);
% EE Jacobian 
Jee = [1,0,... x,y
       l*cos(tB+t1)+l*cos(tB+t1+t2)+l*cos(tB+t1+t2+t3)+l*cos(tB+t1+t2+t3+t4),... tB
   	   l*cos(tB+t1)+l*cos(tB+t1+t2)+l*cos(tB+t1+t2+t3)+l*cos(tB+t1+t2+t3+t4),... t1
   	   l*cos(tB+t1+t2)+l*cos(tB+t1+t2+t3)+l*cos(tB+t1+t2+t3+t4),... t2
       l*cos(tB+t1+t2+t3)+l*cos(tB+t1+t2+t3+t4),... t3
       l*cos(tB+t1+t2+t3+t4); ... t4
       0,1,... x,y
       l*sin(tB+t1)+l*sin(tB+t1+t2)+l*sin(tB+t1+t2+t3)+l*sin(tB+t1+t2+t3+t4),... tB
   	   l*sin(tB+t1)+l*sin(tB+t1+t2)+l*sin(tB+t1+t2+t3)+l*sin(tB+t1+t2+t3+t4),... t1
   	   l*sin(tB+t1+t2)+l*sin(tB+t1+t2+t3)+l*sin(tB+t1+t2+t3+t4),... t2
   	   l*sin(tB+t1+t2+t3)+l*sin(tB+t1+t2+t3+t4),... t3
   	   l*sin(tB+t1+t2+t3+t4)]; % t4

end

