function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

kvz = 7.3;
kpz = 1000;
kvphi = 6.5; 
kpphi = 1000;
kvy = 2.6;
kpy = 162;

u1 = params.mass * (params.gravity - kvz * state.vel(2) + kpz * (des_state.pos(2) - state.pos(2)));
phi_c = (-1 / params.gravity) * (kvy * (-state.vel(1)) + kpy * (des_state.pos(1) - state.pos(1)));
u2 = params.Ixx * (kvphi * (-1 * state.omega(1)) + kpphi * (phi_c - state.rot(1)));


% print state variables for debugging
%state.pos,state.vel,state.rot,state.omega

end

