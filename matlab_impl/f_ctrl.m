classdef f_ctrl < handle
    %F_CTRL Ctrl instance to regulate contact force
    properties
        u_last 
        q_dot_last
        t_last
    end
    
    methods
        function obj = f_ctrl()
            %F_CTRL Construct an instance of this class
            %   Detailed explanation goes here
            obj.u_last = zeros(4,1);
            obj.q_dot_last = zeros(7,1);
            obj.t_last = 0;
        end
        
        function u = ctrl(obj,t, x,dir,f_des,m_base,m_link,r,l,r_tendon)
        %F_CTRL Function that follows a direction until it establishes contact and
        %then tries to regulate the contact force
        % x = Current System State (integral, value, derivative)
        % dir = Desired EE position before contact is established
        % f_des = Desired Contact Force
        % p_des = Current Desired Base Pose
        % m_base = Mass of the base
        % m_link = Mass of one individual link
        % r = Rotor arm length
        % l = Individual Link Length
            
            % Extract state variables 
            q = x(8:14);
            q_dot = x(15:21);
            
            % Numerically differentiate velocity to find acceleration
            q_ddot = (q_dot - obj.q_dot_last)/(t - obj.t_last);

            % Estimate contact force
            f_contact = estimate_force(q,q_dot,q_ddot,obj.u_last,m_base,m_link,r,l,r_tendon);
            
            % Find EE-Pos
            ee = q(1:2);
            R = [cos(q(3)), -sin(q(3));
                 sin(q(3)), cos(q(3))];
            
            for k=1:4
                R = R*[cos(q(3+k)), -sin(q(3+k));
                       sin(q(3+k)), cos(q(3+k))];
                ee = ee + R*[0;-l];
            end

            % We only do force regulation if we are in contact
            if ee(1) >= 3
                % Find the relative position change proportional to force error
                delta_p = 0.1 * (f_contact - f_des)
                u = ctrl(x,[q(1);2] + delta_p,m_base,m_link,r,l);
            % Otherwise we simply do the position control
            else
                u = ctrl(x,dir,m_base,m_link,r,l);
            end

            % Remember values for next iteration
            obj.t_last = t;
            obj.q_dot_last = q_dot;
            obj.u_last = u;

end
    end
end

