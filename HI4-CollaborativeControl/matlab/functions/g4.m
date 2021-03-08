
function u = g4(y, xref, ctrlpar)
% Control function
%     
% Compute the control signal, in this case it is a P-controller which gives
% a control signal proportional to the vector between the current position
% and the reference position. 
%  
% Input arguments:
%   y - measurement y, 
%   xref - the reference vector xref (in this case the desired position of
%          the agent)
%   ctrlpar - dictionary which contains parameters used by the controller 
%             (in this case the proportional gain k).
% 
% Output argument:
%   Control signal 

if numel(xref) == 4
    u =ctrlpar.k_p*(-xref(1:2)+y(1:2)) + ctrlpar.k_v*(-xref(3:4)+y(3:4));
else
    u = ctrlpar.k_p*(y(1:2)-xref(1:2,1)+y(5:6)-xref(1:2,2)) + ...
        ctrlpar.k_v*(y(3:4)-xref(3:4,1)+y(7:8)-xref(3:4,2));
         
        
     

end
