function [ desired_state ] = traj_generator(ti, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
% 
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here
persistent waypoints0 x_t y_t z_t coeff_x coeff_y coeff_z

% create time vector
    t = [0;2;4;8;13];
    %t = [0;1;2.1;4;6];
if nargin > 2
    waypoints0 = waypoints;
    % creating min snap trajectory - 7th order piecewise polynomial
    % 5 waypoints -> 4 polynomials
    % trajectory is a function of time only
    % 
    % each polynomial/coordinate p_i = a_1i + a_2i * t + a_3i * t^2 + .. + a_8i * t^7
    % 
    % CONSTRAINTS
    % 0 -> t_start for p_i
    % 1 -> t_end for p_i
    %
    % at t = 0, p_i = waypoint_i
    % at t = 1, p_i = waypoint_i+1
    % start from rest at waypoint1 and stop at waypoint 5
    % hence, p_1'(t = 0) = 0 aand p_4'(t = 1) = 0
    % all derivatives are continuous at waypoints
    % acceleration and jerk = 0 at start and end
    % p_1''(t = 0) = p_4''(t = 1) = 0
    % p_1'''(t = 0) = p_4'''(t = 1) = 0

    % get x,y,z coordinates of waypoints
    x_t = waypoints0(1,:)';
    y_t = waypoints0(2,:)';
    z_t = waypoints0(3,:)';

    % create trajectory polynomials for x_t, y_t, z_t
    A_x = zeros(32, 32);
    b_x = zeros(32, 1);
    A_y = zeros(32, 32);
    b_y = zeros(32, 1);
    A_z = zeros(32, 32);
    b_z = zeros(32, 1);

    % first set of constraints
    % at t = 0, p_i = waypoint_i
    % at t = 1, p_i = waypoint_i+1

    % time_0_vector(i) = [0 i i^2 i^3 ... i^7]
%     function [time] = time_0_vector(timepoint)
%         time = zeros(1,8);
%         for k = 1:8
%             time(k) = timepoint^(k-1);
%         end            
%     end
    %

    %% known positions and continuity of position
    A_x(1,1:8) = time_0_vector(t(1));
    b_x(1) = x_t(1);

    A_x(2,1:8) = time_0_vector(t(2));
    b_x(2) = x_t(2);
    A_x(3,9:16) = time_0_vector(t(2));
    b_x(3) = x_t(2);

    A_x(4,9:16) = time_0_vector(t(3));
    b_x(4) = x_t(3);
    A_x(5,17:24) = time_0_vector(t(3));
    b_x(5) = x_t(3);

    A_x(6,17:24) = time_0_vector(t(4));
    b_x(6) = x_t(4);
    A_x(7,25:32) = time_0_vector(t(4));
    b_x(7) = x_t(4);

    A_x(8,25:32) = time_0_vector(t(5));
    b_x(8) = x_t(5);
    %%
    A_y(1,1:8) = time_0_vector(t(1));
    b_y(1) = y_t(1);

    A_y(2,1:8) = time_0_vector(t(2));
    b_y(2) = y_t(2);
    A_y(3,9:16) = time_0_vector(t(2));
    b_y(3) = y_t(2);

    A_y(4,9:16) = time_0_vector(t(3));
    b_y(4) = y_t(3);
    A_y(5,17:24) = time_0_vector(t(3));
    b_y(5) = y_t(3);

    A_y(6,17:24) = time_0_vector(t(4));
    b_y(6) = y_t(4);
    A_y(7,25:32) = time_0_vector(t(4));
    b_y(7) = y_t(4);

    A_y(8,25:32) = time_0_vector(t(5));
    b_y(8) = y_t(5);
    %%
    A_z(1,1:8) = time_0_vector(t(1));
    b_z(1) = z_t(1);

    A_z(2,1:8) = time_0_vector(t(2));
    b_z(2) = z_t(2);
    A_z(3,9:16) = time_0_vector(t(2));
    b_z(3) = z_t(2);

    A_z(4,9:16) = time_0_vector(t(3));
    b_z(4) = z_t(3);
    A_z(5,17:24) = time_0_vector(t(3));
    b_z(5) = z_t(3);

    A_z(6,17:24) = time_0_vector(t(4));
    b_z(6) = z_t(4);
    A_z(7,25:32) = time_0_vector(t(4));
    b_z(7) = z_t(4);

    A_z(8,25:32) = time_0_vector(t(5));
    b_z(8) = z_t(5);

    % second set of constraints
    % start from rest at waypoint1 and stop at waypoint 5
    % hence, p_1'(t = 0) = 0 aand p_4'(t = 1) = 0

    % time_1_vector(i) = [0 1 2*i 3*i^2 ... 7*i^6]
%         function [time] = time_1_vector(timepoint)
%             time = zeros(1,8);
%             for k = 2:8
%                 time(k) = (k-1)*timepoint^(k-2);
%             end            
%         end
    %

    % velocities and continuity of velocity
    A_x(9,1:8) = time_1_vector(t(1));
    b_x(9) = 0;

    A_x(10,1:8) = time_1_vector(t(2));
    A_x(10,9:16) = -time_1_vector(t(2));
    b_x(10) = 0;

    A_x(11,9:16) = time_1_vector(t(3));
    A_x(11,17:24) = -time_1_vector(t(3));
    b_x(11) = 0;

    A_x(12,17:24) = time_1_vector(t(4));
    A_x(12,25:32) = -time_1_vector(t(4));
    b_x(12) = 0;

    A_x(13,25:32) = time_1_vector(t(5));
    b_x(13) = 0;

    %%
    A_y(9,1:8) = time_1_vector(t(1));
    b_y(9) = 0;

    A_y(10,1:8) = time_1_vector(t(2));
    A_y(10,9:16) = -time_1_vector(t(2));
    b_y(10) = 0;

    A_y(11,9:16) = time_1_vector(t(3));
    A_y(11,17:24) = -time_1_vector(t(3));
    b_y(11) = 0;

    A_y(12,17:24) = time_1_vector(t(4));
    A_y(12,25:32) = -time_1_vector(t(4));
    b_y(12) = 0;

    A_y(13,25:32) = time_1_vector(t(5));
    b_y(13) = 0;

    %%
    A_z(9,1:8) = time_1_vector(t(1));
    b_z(9) = 0;

    A_z(10,1:8) = time_1_vector(t(2));
    A_z(10,9:16) = -time_1_vector(t(2));
    b_z(10) = 0;

    A_z(11,9:16) = time_1_vector(t(3));
    A_z(11,17:24) = -time_1_vector(t(3));
    b_z(11) = 0;

    A_z(12,17:24) = time_1_vector(t(4));
    A_z(12,25:32) = -time_1_vector(t(4));
    b_z(12) = 0;

    A_z(13,25:32) = time_1_vector(t(5));
    b_z(13) = 0;

    % other derivative related constraints
    % start from rest at waypoint1 and stop at waypoint 5
    % hence, p_1'(t = 0) = 0 aand p_4'(t = 1) = 0

    % time_2_vector(i) = [0 1*0 2*1 3*2*i ... 7*6*i^5]
%         function [time] = time_2_vector(timepoint)
%             time = zeros(1,8);
%             for k = 3:8
%                 time(k) = (k-1)*(k-2)*timepoint^(k-3);
%             end            
%         end
%     %

    % acc and continuity of acc
    A_x(14,1:8) = time_2_vector(t(1));
    b_x(14) = 0;

    A_x(15,1:8) = time_2_vector(t(2));
    A_x(15,9:16) = -time_2_vector(t(2));
    b_x(15) = 0;

    A_x(16,9:16) = time_2_vector(t(3));
    A_x(16,17:24) = -time_2_vector(t(3));
    b_x(16) = 0;

    A_x(17,17:24) = time_2_vector(t(4));
    A_x(17,25:32) = -time_2_vector(t(4));
    b_x(17) = 0;

    A_x(18,25:32) = time_2_vector(t(5));
    b_x(18) = 0;

    %%
    A_y(14,1:8) = time_2_vector(t(1));
    b_y(14) = 0;

    A_y(15,1:8) = time_2_vector(t(2));
    A_y(15,9:16) = -time_2_vector(t(2));
    b_y(15) = 0;

    A_y(16,9:16) = time_2_vector(t(3));
    A_y(16,17:24) = -time_2_vector(t(3));
    b_y(16) = 0;

    A_y(17,17:24) = time_2_vector(t(4));
    A_y(17,25:32) = -time_2_vector(t(4));
    b_y(17) = 0;

    A_y(18,25:32) = time_2_vector(t(5));
    b_y(18) = 0;

    %%
    A_z(14,1:8) = time_2_vector(t(1));
    b_z(14) = 0;

    A_z(15,1:8) = time_2_vector(t(2));
    A_z(15,9:16) = -time_2_vector(t(2));
    b_z(15) = 0;

    A_z(16,9:16) = time_2_vector(t(3));
    A_z(16,17:24) = -time_2_vector(t(3));
    b_z(16) = 0;

    A_z(17,17:24) = time_2_vector(t(4));
    A_z(17,25:32) = -time_2_vector(t(4));
    b_z(17) = 0;

    A_z(18,25:32) = time_2_vector(t(5));
    b_z(18) = 0;

    % time_3_vector(i) = [0 1*0 2*1*0 3*2*1 ... 7*6*5*i^4]
%         function [time] = time_3_vector(timepoint)
%             time = zeros(1,8);
%             for k = 4:8
%                 time(k) = (k-1)*(k-2)*(k-3)*timepoint^(k-4);
%             end            
%         end
    %

    % jerk and continuity of jerk
    A_x(19,1:8) = time_3_vector(t(1));
    b_x(19) = 0;

    A_x(20,1:8) = time_3_vector(t(2));
    A_x(20,9:16) = -time_3_vector(t(2));
    b_x(20) = 0;

    A_x(21,9:16) = time_3_vector(t(3));
    A_x(21,17:24) = -time_3_vector(t(3));
    b_x(21) = 0;

    A_x(22,17:24) = time_3_vector(t(4));
    A_x(22,25:32) = -time_3_vector(t(4));
    b_x(22) = 0;

    A_x(23,25:32) = time_3_vector(t(5));
    b_x(23) = 0;

    %%
    A_y(19,1:8) = time_3_vector(t(1));
    b_y(19) = 0;

    A_y(20,1:8) = time_3_vector(t(2));
    A_y(20,9:16) = -time_3_vector(t(2));
    b_y(20) = 0;

    A_y(21,9:16) = time_3_vector(t(3));
    A_y(21,17:24) = -time_3_vector(t(3));
    b_y(21) = 0;

    A_y(22,17:24) = time_3_vector(t(4));
    A_y(22,25:32) = -time_3_vector(t(4));
    b_y(22) = 0;

    A_y(23,25:32) = time_3_vector(t(5));
    b_y(23) = 0;

    %%
    A_z(19,1:8) = time_3_vector(t(1));
    b_z(19) = 0;

    A_z(20,1:8) = time_3_vector(t(2));
    A_z(20,9:16) = -time_3_vector(t(2));
    b_z(20) = 0;

    A_z(21,9:16) = time_3_vector(t(3));
    A_z(21,17:24) = -time_3_vector(t(3));
    b_z(21) = 0;

    A_z(22,17:24) = time_3_vector(t(4));
    A_z(22,25:32) = -time_3_vector(t(4));
    b_z(22) = 0;

    A_z(23,25:32) = time_3_vector(t(5));
    b_z(23) = 0;

    % time_4_vector(i) = [0 1*0 2*1*0 3*2*1*0 ... 7*6*5*i^4]
%         function [time] = time_4_vector(timepoint)
%             time = zeros(1,8);
%             for k = 5:8
%                 time(k) = (k-1)*(k-2)*(k-3)*(k-4)*timepoint^(k-5);
%             end            
%         end
    %

    % continuity of snap

    A_x(24,1:8) = time_4_vector(t(2));
    A_x(24,9:16) = -time_4_vector(t(2));
    b_x(24) = 0;

    A_x(25,9:16) = time_4_vector(t(3));
    A_x(25,17:24) = -time_4_vector(t(3));
    b_x(25) = 0;

    A_x(26,17:24) = time_4_vector(t(4));
    A_x(26,25:32) = -time_4_vector(t(4));
    b_x(26) = 0;

    %%
    A_y(24,1:8) = time_4_vector(t(2));
    A_y(24,9:16) = -time_4_vector(t(2));
    b_y(24) = 0;

    A_y(25,9:16) = time_4_vector(t(3));
    A_y(25,17:24) = -time_4_vector(t(3));
    b_y(25) = 0;

    A_y(26,17:24) = time_4_vector(t(4));
    A_y(26,25:32) = -time_4_vector(t(4));
    b_y(26) = 0;

    %%
    A_z(24,1:8) = time_4_vector(t(2));
    A_z(24,9:16) = -time_4_vector(t(2));
    b_z(24) = 0;

    A_z(25,9:16) = time_4_vector(t(3));
    A_z(25,17:24) = -time_4_vector(t(3));
    b_z(25) = 0;

    A_z(26,17:24) = time_4_vector(t(4));
    A_z(26,25:32) = -time_4_vector(t(4));
    b_z(26) = 0;

    % time_5_vector(i) = [0 1*0 2*1*0 3*2*1*0 ... 7*6*5*4*3*i^2]
%         function [time] = time_5_vector(timepoint)
%             time = zeros(1,8);
%             for k = 6:8
%                 time(k) = (k-1)*(k-2)*(k-3)*(k-4)*(k-5)*timepoint^(k-6);
%             end            
%         end
    %

    % continuity of 5th derivatives

    A_x(27,1:8) = time_5_vector(t(2));
    A_x(27,9:16) = -time_5_vector(t(2));
    b_x(27) = 0;

    A_x(28,9:16) = time_5_vector(t(3));
    A_x(28,17:24) = -time_5_vector(t(3));
    b_x(28) = 0;

    A_x(29,17:24) = time_5_vector(t(4));
    A_x(29,25:32) = -time_5_vector(t(4));
    b_x(29) = 0;

    %%
    A_y(27,1:8) = time_5_vector(t(2));
    A_y(27,9:16) = -time_5_vector(t(2));
    b_y(27) = 0;

    A_y(28,9:16) = time_5_vector(t(3));
    A_y(28,17:24) = -time_5_vector(t(3));
    b_y(28) = 0;

    A_y(29,17:24) = time_5_vector(t(4));
    A_y(29,25:32) = -time_5_vector(t(4));
    b_y(29) = 0;

    %%
    A_z(27,1:8) = time_5_vector(t(2));
    A_z(27,9:16) = -time_5_vector(t(2));
    b_z(27) = 0;

    A_z(28,9:16) = time_5_vector(t(3));
    A_z(28,17:24) = -time_5_vector(t(3));
    b_z(28) = 0;

    A_z(29,17:24) = time_5_vector(t(4));
    A_z(29,25:32) = -time_5_vector(t(4));
    b_z(29) = 0;

    % time_6_vector(i) = [0 1*0 2*1*0 3*2*1*0 ... 7*6*5*4*3*2*i]
%         function [time] = time_6_vector(timepoint)
%             time = zeros(1,8);
%             for k = 7:8
%                 time(k) = (k-1)*(k-2)*(k-3)*(k-4)*(k-5)*(k-6)*timepoint^(k-7);
%             end            
%         end
    %

    % continuity of 6th derivatives

    A_x(30,1:8) = time_6_vector(t(2));
    A_x(30,9:16) = -time_6_vector(t(2));
    b_x(30) = 0;

    A_x(31,9:16) = time_6_vector(t(3));
    A_x(31,17:24) = -time_6_vector(t(3));
    b_x(31) = 0;

    A_x(32,17:24) = time_6_vector(t(4));
    A_x(32,25:32) = -time_6_vector(t(4));
    b_x(32) = 0;

    %%
    A_y(30,1:8) = time_6_vector(t(2));
    A_y(30,9:16) = -time_6_vector(t(2));
    b_y(30) = 0;

    A_y(31,9:16) = time_6_vector(t(3));
    A_y(31,17:24) = -time_6_vector(t(3));
    b_y(31) = 0;

    A_y(32,17:24) = time_6_vector(t(4));
    A_y(32,25:32) = -time_6_vector(t(4));
    b_y(32) = 0;

    %%
    A_z(30,1:8) = time_6_vector(t(2));
    A_z(30,9:16) = -time_6_vector(t(2));
    b_z(30) = 0;

    A_z(31,9:16) = time_6_vector(t(3));
    A_z(31,17:24) = -time_6_vector(t(3));
    b_z(31) = 0;

    A_z(32,17:24) = time_6_vector(t(4));
    A_z(32,25:32) = -time_6_vector(t(4));
    b_z(32) = 0;

    % solve for coefficients
    coeff_x = (A_x \ b_x)';
    coeff_y = (A_y \ b_y)';
    coeff_z = (A_z \ b_z)';

%% generate desired states, veloctites and accelerations
else
    if ((ti >= t(1)) && (ti <= t(2)))
        desired_state.pos(1) = coeff_x(1:8) * time_0_vector(ti)';
        desired_state.pos(2) = coeff_y(1:8) * time_0_vector(ti)';
        desired_state.pos(3) = coeff_z(1:8) * time_0_vector(ti)';
        desired_state.vel(1) = coeff_x(1:8) * time_1_vector(ti)';
        desired_state.vel(2) = coeff_y(1:8) * time_1_vector(ti)';
        desired_state.vel(3) = coeff_z(1:8) * time_1_vector(ti)';
        desired_state.acc(1) = coeff_x(1:8) * time_2_vector(ti)';
        desired_state.acc(2) = coeff_y(1:8) * time_2_vector(ti)';
        desired_state.acc(3) = coeff_z(1:8) * time_2_vector(ti)';
    elseif ((ti >= t(2)) && (ti <= t(3)))
        desired_state.pos(1) = coeff_x(9:16) * time_0_vector(ti)';
        desired_state.pos(2) = coeff_y(9:16) * time_0_vector(ti)';
        desired_state.pos(3) = coeff_z(9:16) * time_0_vector(ti)';
        desired_state.vel(1) = coeff_x(9:16) * time_1_vector(ti)';
        desired_state.vel(2) = coeff_y(9:16) * time_1_vector(ti)';
        desired_state.vel(3) = coeff_z(9:16) * time_1_vector(ti)';
        desired_state.acc(1) = coeff_x(9:16) * time_2_vector(ti)';
        desired_state.acc(2) = coeff_y(9:16) * time_2_vector(ti)';
        desired_state.acc(3) = coeff_z(9:16) * time_2_vector(ti)';
    elseif ((ti >= t(3)) && (ti <= t(4)))
        desired_state.pos(1) = coeff_x(17:24) * time_0_vector(ti)';
        desired_state.pos(2) = coeff_y(17:24) * time_0_vector(ti)';
        desired_state.pos(3) = coeff_z(17:24) * time_0_vector(ti)';
        desired_state.vel(1) = coeff_x(17:24) * time_1_vector(ti)';
        desired_state.vel(2) = coeff_y(17:24) * time_1_vector(ti)';
        desired_state.vel(3) = coeff_z(17:24) * time_1_vector(ti)';
        desired_state.acc(1) = coeff_x(17:24) * time_2_vector(ti)';
        desired_state.acc(2) = coeff_y(17:24) * time_2_vector(ti)';
        desired_state.acc(3) = coeff_z(17:24) * time_2_vector(ti)';
    elseif ((ti >= t(4)) && (ti <= t(5)))
        desired_state.pos(1) = coeff_x(25:32) * time_0_vector(ti)';
        desired_state.pos(2) = coeff_y(25:32) * time_0_vector(ti)';
        desired_state.pos(3) = coeff_z(25:32) * time_0_vector(ti)';
        desired_state.vel(1) = coeff_x(25:32) * time_1_vector(ti)';
        desired_state.vel(2) = coeff_y(25:32) * time_1_vector(ti)';
        desired_state.vel(3) = coeff_z(25:32) * time_1_vector(ti)';
        desired_state.acc(1) = coeff_x(25:32) * time_2_vector(ti)';
        desired_state.acc(2) = coeff_y(25:32) * time_2_vector(ti)';
        desired_state.acc(3) = coeff_z(25:32) * time_2_vector(ti)';
    else
        desired_state.pos(1) = coeff_x(25:32) * time_0_vector(t(5))';
        desired_state.pos(2) = coeff_y(25:32) * time_0_vector(t(5))';
        desired_state.pos(3) = coeff_z(25:32) * time_0_vector(t(5))';
        desired_state.vel(1) = coeff_x(25:32) * time_1_vector(t(5))';
        desired_state.vel(2) = coeff_y(25:32) * time_1_vector(t(5))';
        desired_state.vel(3) = coeff_z(25:32) * time_1_vector(t(5))';
        desired_state.acc(1) = coeff_x(25:32) * time_2_vector(t(5))';
        desired_state.acc(2) = coeff_y(25:32) * time_2_vector(t(5))';
        desired_state.acc(3) = coeff_z(25:32) * time_2_vector(t(5))';
        
    end
    desired_state.pos = desired_state.pos';
    desired_state.vel = desired_state.vel';
    desired_state.acc = desired_state.acc';

end

desired_state.yaw = 0;
desired_state.yawdot = 0;

end

function [time] = time_0_vector(timepoint)
    time = zeros(1,8);
    for k = 1:8
        time(k) = timepoint^(k-1);
    end            
end

function [time] = time_1_vector(timepoint)
    time = zeros(1,8);
    for k = 2:8
        time(k) = (k-1)*timepoint^(k-2);
    end            
end

function [time] = time_2_vector(timepoint)
    time = zeros(1,8);
    for k = 3:8
        time(k) = (k-1)*(k-2)*timepoint^(k-3);
    end            
end

function [time] = time_3_vector(timepoint)
    time = zeros(1,8);
    for k = 4:8
        time(k) = (k-1)*(k-2)*(k-3)*timepoint^(k-4);
    end            
end

function [time] = time_4_vector(timepoint)
    time = zeros(1,8);
    for k = 5:8
        time(k) = (k-1)*(k-2)*(k-3)*(k-4)*timepoint^(k-5);
    end            
end

function [time] = time_5_vector(timepoint)
time = zeros(1,8);
    for k = 6:8
        time(k) = (k-1)*(k-2)*(k-3)*(k-4)*(k-5)*timepoint^(k-6);
    end
end

function [time] = time_6_vector(timepoint)
    time = zeros(1,8);
    for k = 7:8
        time(k) = (k-1)*(k-2)*(k-3)*(k-4)*(k-5)*(k-6)*timepoint^(k-7);
    end            
end











