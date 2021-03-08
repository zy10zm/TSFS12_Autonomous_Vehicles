classdef StateFeedbackController < ControllerBase
    properties
        K;
        L;
        plan;
        goal_tol;
        state;
        s0;
    end
    methods
        function obj=StateFeedbackController(K, L, path, goal_tol)
            if nargin < 4
                goal_tol = 1;
            end
            obj = obj@ControllerBase();
            obj.K = K;  % Feedback gain
            obj.L = L;  % Vehicle wheel base
            obj.plan = path;  % Path to follow
            obj.goal_tol = goal_tol;  % Goal tolerance
            obj.s0 = 0.0;  % Path position state
        end

        function theta_e = heading_error(obj, theta, s)
            % theta, heading of vehicle
            % s - position (length) on path
            % Compute heading error. The SplinePath method heading is useful
            
            % YOUR CODE HERE
            [h_s,normal] = obj.plan.heading(s);
            h = [cos(theta),sin(theta)]; %Kanske måste plussa på pi/2
            help_sin = cross([h_s,0],[h,0])/(norm(h_s)*norm(h));
            help_cos = dot(h_s,h)/(norm(h_s)*norm(h));
            theta_e = atan2(help_sin(3),help_cos); %0.0;
        end

        function c = u(obj, t, w)
            theta = w(3);
            p_car = w(1:2);
            
            % Compute d and theta_e errors. Use the SplinePath method project
            % and the obj.heading_error() function you've written above

            % YOUR CODE HERE
            ds = 0.01;
            s_lim = 0.2;
            [s,d] = obj.plan.project(p_car,obj.s0,ds,s_lim);
            obj.s0=s;
            theta_e = obj.heading_error(theta,s);

            % Compute control signal delta
            acc = 0;  % Constant speed
            u0 = obj.plan.c(s+1) - obj.K(1)*d-obj.K(2)*theta_e;
            delta = atan2(u0*obj.L,1); % Steering angle
            c = [delta, acc];
        end

        function r = run(obj, t, w)
            % Function that returns true until goal is reached
            p_car = w(1:2);
            p_goal = obj.plan.path(end,:);
            
            dp = p_car - p_goal;
            dist = norm(dp);
            if dist < obj.goal_tol
                r = false;
            else
                r = true;
            end
            %r = norm(p_car - p_goal) > obj.goal_tol;
        end
    end
end
