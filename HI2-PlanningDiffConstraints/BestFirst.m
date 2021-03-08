function res = BestFirst(num_nodes, mission, f, heuristic, num_controls)
    if nargin < 5
        num_controls = 0;
    end
    previous = zeros(num_nodes, 1);
    control_to_come = zeros(num_nodes, num_controls);
    cost_to_come = zeros(num_nodes, 1);
    tic;
    q = PriorityQueue();
    q.insert(heuristic(mission.start.id,mission.goal.id), mission.start.id);

    foundPlan = false;
    while ~q.isempty()
        [~,x] = q.pop();
        if x == mission.goal.id
            foundPlan = true;
            break;
        end
        [neighbours, u, d] = f(x);
        for k=1:numel(neighbours)
            xi = neighbours(k);
            di = d(k);
            ui = u(k, :);
            if previous(xi) == 0
                previous(xi) = x;
                cost_to_come(xi) = cost_to_come(x) + di;
                q.insert(heuristic(xi,mission.goal.id),xi);
                if num_controls > 0
                    control_to_come(xi, :) = ui;
                end
            end
        end
    end

    % Collect plan by traversing visited nodes
    if ~foundPlan
        plan = [];
        control = [];
        length = 0;
    else 
        % collect the plan
        plan = [mission.goal.id];
        control = [];
        length = cost_to_come(mission.goal.id);
        while plan(1) ~= mission.start.id
            control = [control_to_come(plan(1),:);control];      
            plan = [previous(plan(1)) plan];
        end
    end
    res.time = toc;
    res.plan = plan;
    res.length = length;
    res.num_visited_nodes = sum(previous > 0);
    res.visited_nodes = previous(previous>0);
    res.name = 'BestFirst';
    res.control = control;
end