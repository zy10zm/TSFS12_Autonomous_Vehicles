function dxdt = multi_agent_ode(t,x,agent,n)

% Reshape the state vector to a 2x4 matrix, where the columns are the
% positions of the four agents.

xr = reshape(x,n,[]);
dxdt =[];

for i = 1:length(agent)
    % Measure y = h(x)
    y = agent(i).h(x, agent(i).measpar);
    
    % Compute the control signal u = g(y, reference value)
    u = agent(i).g(y, agent(i).xref(t), agent(i).ctrlpar);
    %u = [0;0]
    % Compute the right-hand side in the model x' = f(x,u) for each agent
    % and add to the vector dxdt'
    dxdt =  [dxdt;agent(i).f(t, xr(:,i), u, agent(i).mdlpar)];
end
end