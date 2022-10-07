classdef force_controller
    %FORCE_CONTROLLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        e_f_integral,
        e_f_last,
        x_last,
        p_last,
        t_last,

        m_base, m_link, r, l, r_tendon
    end
    
    methods
        function obj = force_controller(m_base, m_link, r, l, r_tendon)
            %FORCE_CONTROLLER Construct an instance of this class
            %   Detailed explanation goes here
            
            % Save system parameters
            obj.m_base = m_base;
            obj.m_link = m_link;
            obj.r = r;
            obj.l = l;
            obj.r_tendon = r_tendon;

            % Init other values to zero
            obj.e_f_integral = 0.0;
            obj.e_f_last = 0.0;
            obj.x_last = zeros(21,1);
            obj.p_last = 0.0;
            obj.t_last = -100;
        end

        function p = ref_pos(obj, f_des, x, t)
            %REF_POS Summary of this function goes here
            %   Detailed explanation goes here
            
            % Find Timestep
            dt = t-obj.t_last;

            % Define PID constants
            kp = 0.5;
            kd = 15;
            ki = 0.05;
            
            % Extracting state variables
            q = x(8:14);
            q_dot = x(15:21);
            q_ddot = (x(15:21) - obj.x_last(15:21))/dt;
            obj.x_last = x;
    
            % Compute currently active control
            u_last = ctrl(obj.x_last,obj.t_last, obj.p_last,obj.m_base,obj.r);

            % Estimate Contact Forc
            f_est = estimate_force(q, q_dot,...
                                   q_ddot, u_last, ...
                                   obj.m_base,obj.m_link,obj.r,...
                                   obj.l,obj.r_tendon);
            
            % Find force error
            e_f = f_des - f_est(1);
            % Its derivative
            de_f = (e_f - obj.e_f_last) / dt;
            % And the integral
            obj.e_f_integral = obj.e_f_integral + e_f*dt;
            obj.e_f_last = e_f;

            % Find reference pos
            obj.p_last = x(4)-(kp * e_f + kd * de_f + ki * obj.e_f_integral);
            obj.t_last = t;
            % Return Value
            p = obj.p_last;


        end
    end
end

