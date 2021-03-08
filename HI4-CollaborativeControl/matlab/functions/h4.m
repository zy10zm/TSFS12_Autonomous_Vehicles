
function y = h4(x, measpar, i)
% Measurement function
%
% The measurement function gives the measurements available to the agent.
%
% Input arguments:
%   x - the complete state vector of all agents in the multi-agent system
%   measpar - structure containing parameters used in the function. In
%             this case the indices in the state vector x of the states
%             measured by the agent stored in meas_idx.
%
% Output:
%   y - The measurement vector

% GER UT p_j-p_i resp v_j-v_i
tmp = x(measpar.idx);
if numel(tmp) == 8;
    y = [tmp(2)-tmp(1);tmp(4)-tmp(3);tmp(6)-tmp(5);tmp(8)-tmp(7)];
else
    y = [tmp(2)-tmp(1);tmp(4)-tmp(3);tmp(6)-tmp(5);tmp(8)-tmp(7);...
        tmp(10)-tmp(9);tmp(12)-tmp(11);tmp(14)-tmp(13);tmp(16)-tmp(15)];
end