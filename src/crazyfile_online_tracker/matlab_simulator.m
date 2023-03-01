% Load simulink model
%define parameters for Simulink model
cmd_2_newtons_conversion_quadratic_coefficient=  1.3385e-10;
cmd_2_newtons_conversion_linear_coefficient     =  6.4870e-6;
nrotor_vehicle_thrust_max = 0.1597;
g=9.81;
CL_sys = get_CL_model(g);
state_initial= [0,0,0.3,0,0,0,0,0,0]; % start simulation after taking off
x_initial = [state_initial zeros(1, 7)]; % 7 initial states for integrators in controller, all start from 0

% Initialize time
sim_t = 0;
sim_timestep = 0.01;

% Initialize plot
fig = figure;
hold on
ax_x = subplot(4,2,1);
ax_y = subplot(4,2,3);
ax_z = subplot(4,2,5);
ax_yaw = subplot(4,2,7);
ax_BEV = subplot(4,2,[2,4,6,8]);
axes = [ax_x, ax_y, ax_z, ax_yaw, ax_BEV];

% Initialize ROS node
masterHost = 'http://localhost:11311';
rosinit(masterHost)
setpoint_sub = rossubscriber('/setpointHL','crazyflie_online_tracker/SetpointHL', ...
    'DataFormat','struct');
sim_state_pub = rospublisher('/crazyflieState','crazyflie_online_tracker/CrazyflieState',...
    'DataFormat','struct');

% Send initial state
curr_state = state_initial;
curr_state_smg = vector_to_state_msg(sim_state_pub, curr_state);
send(sim_state_pub, curr_state_smg);

% Start simulation
STATUSTEXT = 'success';
t = [0, 0.05]; % [s] simulation step size
timeout = 0.5; % [s] subscriber timeout
while(strcmp(STATUSTEXT,'success'))
    [message, STATUS, STATUSTEXT] = receive(setpoint_sub, timeout);
    if ~strcmp(STATUSTEXT,'success') % if no msg is received
        for i=1:5 % resend state message for at most 5 times
            send(sim_state_pub, curr_state_smg); 
            [message, STATUS, STATUSTEXT] = receive(setpoint_sub, timeout);
            if strcmp(STATUSTEXT,'success')
                break
            end
        end
    end
    if strcmp(STATUSTEXT,'success')
        reference = zeros(2, 9);
        reference(:,4) = message.Velocity.X;
        reference(:,5) = message.Velocity.Y;
        reference(:,3) = message.Position.Z;
        reference(:,9) = message.Yaw;
        [output_sim, ~, x_sim] = lsim(CL_sys, reference, t, x_initial);
        next_state = output_sim(end, :);
        x_initial = x_sim(end, :);
        sim_t = sim_t + sim_timestep;
        plot_trajectory(axes, reference(end, :), curr_state, next_state, sim_t, sim_timestep);
        curr_state = next_state;
        curr_state_smg = vector_to_state_msg(sim_state_pub, curr_state);
        send(sim_state_pub, curr_state_smg);
    end
end
rosshutdown;

function state_msg = vector_to_state_msg(sim_state_pub, state)
    state_msg = rosmessage(sim_state_pub);
    state_msg.Pose.Position.X = state(1);
    state_msg.Pose.Position.Y = state(2);
    state_msg.Pose.Position.Z = state(3);
    quat = angle2quat(state(9), state(8), state(7));
    state_msg.Pose.Orientation.X = quat(1);
    state_msg.Pose.Orientation.Y = quat(2);
    state_msg.Pose.Orientation.Z = quat(3);
    state_msg.Pose.Orientation.W = quat(4);
    state_msg.Velocity.X = state(4);
    state_msg.Velocity.Y = state(5);
    state_msg.Velocity.Z = state(6);
end