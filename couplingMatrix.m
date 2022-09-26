function [H] = couplingMatrix(q,m_base,m_link,r,l)
%COUPLINGMATRIX Function that returns the state dependent dynamic coupling
%term between the base and the arm

% Extract Joint variables
t1 = q(4);
t2 = q(5);
t3 = q(6);
t4 = q(7);


H = [
    1/2*l*m_link*(7*cos(t1)+5*cos(t1+t2)+3*cos(t1+t2+t3)+cos(t1+t2+t3+t4)),...
    1/2*l*m_link*(5*cos(t1+t2)+3*cos(t1+t2+t3)+cos(t1+t2+t3+t4)),...
    1/2*l*m_link*(3*cos(t1+t2+t3)+cos(t1+t2+t3+t4)),...
    1/2*l*m_link*cos(t1+t2+t3+t4); % Coupling effect for x-axis

    -(1/2)*l*m_link*(7*sin(t1)+5*sin(t1+t2)+3*sin(t1+t2+t3)+sin(t1+t2+t3+t4)),...
    -(1/2)*l*m_link*(5*sin(t1+t2)+3*sin(t1+t2+t3)+sin(t1+t2+t3+t4)),...
    -(1/2)*l*m_link*(3*sin(t1+t2+t3)+sin(t1+t2+t3+t4)),...
    -(1/2)*l*m_link*sin(t1+t2+t3+t4); % Coupling effect for y-axis

    (4*l^2*m_link)/3,...
    -(1/2)*l^2*m_link*(-2+5*sin(t2)+3*sin(t2+t3)+sin(t2+t3+t4)),...
    -(1/6)*l^2*m_link*(-4+9*sin(t3)+9*sin(t2+t3)+3*sin(t3+t4)+3*sin(t2+t3+t4)),...
    -(1/6)*l^2*m_link*(-2+3*sin(t4)+3*sin(t3+t4)+3*sin(t2+t3+t4));
    ]; % Coupling effect for rotation

end

