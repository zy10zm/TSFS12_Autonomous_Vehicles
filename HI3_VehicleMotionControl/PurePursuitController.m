classdef PurePursuitController < ControllerBase
    properties
        l;
        L;
        plan;
        goal_tol;
        s;
    end
    methods
        function obj=PurePursuitController(l, L, path, goal_tol)
            obj = obj@ControllerBase();
            if nargin < 4
                goal_tol = 1;
            end
            obj.l = l;
            obj.L = L;
            obj.plan = path;
            obj.goal_tol = goal_tol;
            obj.s = 1;
        end

        function p_purepursuit = pursuit_point(obj, p_car, theta)
            s = obj.s; % Last stored path parameter
            path_points = obj.plan.path(s:end,:);  % Points 
            l = obj.l;  % Pure-pursuit look-ahead
%           Your code here
      
            D = pdist2(path_points, p_car);
            D_l = min(abs(l-D));
            [row_of_path_points,~] = find(abs(D-l) == D_l);
            
            obj.s = row_of_path_points;
            Px = path_points(row_of_path_points,1); 
            Py = path_points(row_of_path_points,2);
            P_purepursuit = [Px Py]; %global coords for intersection point 

            % vehicle local coords for intersection point
            py = (Px-p_car(1))*cos(theta) + (Py-p_car(2))*sin(theta);
            px = -(Px-p_car(1))*sin(theta) + (Py-p_car(2))*cos(theta);

            p_purepursuit = [px py];
        end
        
        function delta = pure_pursuit_control(obj, dp, theta)        
            delta = atan(obj.L*2*dp(1)/obj.l^2); 
        end

        function c = u(obj, t, w)
            p_car = w(1:2);
            theta = w(3);      
            p_intersection = pursuit_point(obj, p_car, theta);
            delta = pure_pursuit_control(obj, p_intersection, theta);
            % Your code here to compute steering angle, use the functions
            % obj.pursuit_point() and obj.pure_pursuit_control() you 
            % have written above.
            acc = 0;
           
            c = [delta, acc];
        end
    
        function r = run(obj, t, w)
            % Function that returns true until goal is reached
            p_car = w(1:2);
            p_goal = obj.plan.path(end, :);
            r = norm(p_car - p_goal) > obj.goal_tol;
        end
    end
end
