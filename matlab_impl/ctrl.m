classdef ctrl < handle
    %F_CTRL Ctrl instance to regulate contact force
    properties
        u_last 
        q_dot_last
        t_last
        f_last
        e_f_last
        established_contact;
    end
    
    methods
        function obj = ctrl()
            %F_CTRL Construct an instance of this class
            %   Detailed explanation goes here
            obj.u_last = zeros(4,1);
            obj.q_dot_last = zeros(7,1);
            obj.t_last = 0;
            obj.f_last = zeros(2,1);
            obj.e_f_last = zeros(2,1);
            obj.established_contact = false;
        end
        
        function u = pos_ctrl(obj,x,p_des,m_base,m_link, r,l)
        %POS_CTRL Chains the various controllers to compute the position
        %control action.
        % x = Current System State (integral, value, derivative)
        % p_des = Current Desired Base Pose
        % m_base = Mass of the base
        % m_link = Mass of one individual link
        % r = Rotor arm length
        % l = Individual Link Length        
            % Desired base position
            p_base = ee2base(p_des,x(11:14),l);
            
            % Control Action
            u = [max(min(angle_lqri(x, m_base, m_link, r, l,...
                                      pos_pd(x, p_base(1)) ...
                                         )...
                            + height_lqr(x, m_base + 4*m_link, p_base(2)),...
                            [5;5]),...
                           0);    % Bi-Rotor Inputs
                           [0;5]... Tendon Inputs
                        ];
        
        
        end

        function p = f_p_ref(obj,t,x,dir,f_des,m_base,m_link,r,l,r_tendon)
        % F_P_REF Function that computes the reference position given the
        % current state and therefore the infered force tracking error.
        % t = current time stamp
        % x = current state
        % dir = target location until we're in contact
        % f_des = desired force 

            % Extract state variables 
            q = x(8:14);
            q_dot = x(15:21);
            
            % Numerically differentiate velocity to find acceleration
            % Handle edge case of first measurement
            if t ~= 0
                q_ddot = (q_dot - obj.q_dot_last)/(t - obj.t_last);
            else 
                q_ddot = zeros(7,1);
            end
            % Estimate contact force
            f_contact = estimate_force(q,q_dot,q_ddot,obj.u_last,m_base,...
                                       m_link,r,l,r_tendon);
            
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
            if (ee(1) >= 3) || (obj.established_contact)
                % Find the force error
                e_f = f_des - f_contact;
                % Rate of change of force error
                e_f_dot = (e_f - obj.e_f_last)/(t-obj.t_last);
                % Find the relative position change PD to force error
                delta_p = 1 * e_f + 0.001 * e_f_dot;
                % Reference position
                p = [dir(1) - delta_p(1);1];

                % Remember last values
                obj.f_last = f_contact;
                obj.e_f_last = e_f;

                % First time we get here we establish contact
                obj.established_contact = true;
                                   
            % Otherwise we simply keep following the previous targe
            else
                p = dir;
            end 

        end

        function [u,p] = f_ctrl(obj,t, x,dir,f_des,m_base,m_link,r,l,r_tendon)
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
            
            % Enforce Control Frequency
            if t - obj.t_last > 0.001
                 % Extract state variables 
                q_dot = x(15:21);

                p = obj.f_p_ref(t,x,dir,f_des,m_base,m_link,r,l,r_tendon);
                u = obj.pos_ctrl(x,p,m_base,m_link,r,l);   

                % Remember values for next iteration
                obj.t_last = t;
                obj.q_dot_last = q_dot;
                obj.u_last = u;
            else
                u = obj.u_last;
                p = dir;
            end
        end
    end
end

