function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%  

% position and velocity errors
e_p = des_state.pos - state.pos;
e_v = des_state.vel - state.vel;

% gains
k_p = [30; 30; 120];
k_v = [8; 8; 15];

% commanded acceleration
commanded.acc = des_state.acc + k_v .* e_v+ k_p .* e_p;

% input u1 = F
F = params.mass * (params.gravity + commanded.acc(3));

% desired roll and pitch
des_phi= ((commanded.acc(1) * sin(des_state.yaw)) - (commanded.acc(2) * cos(des_state.yaw))) / params.gravity ;
des_theta = ((commanded.acc(1) * cos(des_state.yaw)) + (commanded.acc(2) * sin(des_state.yaw))) / params.gravity ;

% gains
k_ap = [10; 10; 5];
k_omega = [2; 2; 1];

% desired orientation and angular velocity
des_state.rot = [des_phi; des_theta; des_state.yaw];
des_state.omega = [0; 0; des_state.yawdot];

% input u2 = M
M = k_ap .* (des_state.rot - state.rot) + k_omega .* (des_state.omega - state.omega);

% =================== Your code ends here ===================

end
