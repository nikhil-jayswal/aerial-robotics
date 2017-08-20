function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% set proportional gain
k_p = 100;

% set derivative gain
k_v = 12;

% calculate error and v_error = d(error)/dt
error = s_des(1) - s(1);
v_error = s_des(2) - s(2);

% return required input
% ddot(z_des) = 0 (hover at constant altitude)
u = params.mass * (k_p * error + k_v * v_error + params.gravity);

end

