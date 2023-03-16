% if the simulation doesn't start from t=0, run this command manually and
% try again.
clear all

%% Initialization
global sim_timestep nrotor_initial_condition reference
sim_timestep = 0.1;

% Initial conditions for the full state
nrotor_initial_condition_p       =    [0; 0; 0.3]; % start simulation after taking off
nrotor_initial_condition_p_dot   =    [0; 0; 0];
nrotor_initial_condition_psi     =    [0; 0; 0];
nrotor_initial_condition_psi_dot =    [0; 0; 0];
nrotor_initial_condition = [...
        nrotor_initial_condition_p          ;...
        nrotor_initial_condition_p_dot      ;...
        nrotor_initial_condition_psi        ;...
        nrotor_initial_condition_psi_dot    ;...
    ];
% Initial conditions for the body rates
nrotor_initial_condition_body_rates = [0; 0; 0];

% Initialize plot
fig = figure;
hold on
ax_x = subplot(4,2,1);
ax_y = subplot(4,2,3);
ax_z = subplot(4,2,5);
ax_yaw = subplot(4,2,7);
ax_BEV = subplot(4,2,[2,4,6,8]);
global axes
axes = [ax_x, ax_y, ax_z, ax_yaw, ax_BEV];

% Initialize ROS node
masterHost = 'http://localhost:11311';
rosinit(masterHost)
setpoint_sub = rossubscriber('/setpointHL','crazyflie_online_tracker/SetpointHL', @setpoint_sub_callback, ...
    'DataFormat','struct','BufferSize', 100);
global sim_state_pub
sim_state_pub = rospublisher('/crazyflieState','crazyflie_online_tracker/CrazyflieState',...
    'DataFormat','struct');

% Send initial state
curr_state = nrotor_initial_condition';
curr_state_smg = vector_to_state_msg(sim_state_pub, curr_state);
send(sim_state_pub, curr_state_smg);

%% Start simulation
% put a break point at this line so that the script doesn't terminate 
% immediately(works as rospy.spin())
rosshutdown; 

function setpoint_sub_callback(~, message)
    global sim_timestep nrotor_initial_condition reference axes sim_state_pub
    persistent sim_t finalOP curr_state
    if isempty(sim_t)
        sim_t = 0;
    end
    if isempty(curr_state)
        curr_state = nrotor_initial_condition';
    end
    reference = [0, ...
                 message.Velocity.X, ...
                 message.Velocity.Y, ...
                 message.Position.Z, ...
                 message.Yaw]; % time, vx_ref, vy_ref, z_ref, yaw_ref
    simIn = Simulink.SimulationInput('nonlinear_model');
    simIn = setModelParameter(simIn,"StopTime",string(sim_t + sim_timestep),...
        "SimulationMode", "normal");
    if sim_t>0
        simIn = setInitialState(simIn,finalOP);
    end
    simOut = sim(simIn);
    finalOP = simOut.finalOP;
    next_state = simOut.next_state(end,:);
    sim_t = sim_t + sim_timestep;
    plot_trajectory(axes, reference(2:end), curr_state, next_state, sim_t, sim_timestep);
    curr_state = next_state;
    curr_state_smg = vector_to_state_msg(sim_state_pub, curr_state);
    send(sim_state_pub, curr_state_smg);
end

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
    state_msg.Velocity.Linear.X = state(4);
    state_msg.Velocity.Linear.Y = state(5);
    state_msg.Velocity.Linear.Z = state(6);
    state_msg.Velocity.Angular.X = state(10);
    state_msg.Velocity.Angular.Y = state(11);
    state_msg.Velocity.Angular.Z = state(12);
end