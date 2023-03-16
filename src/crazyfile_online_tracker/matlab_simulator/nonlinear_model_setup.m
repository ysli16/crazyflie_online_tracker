% Gravitational constant:
g = 9.81;
g_vec = [0; 0; -g];


% The TRUE mass of the N-rotor vehicle, used for simulating the equations
% of motion [kg]
nrotor_vehicle_mass_true = 32e-3;
% The "measured" mass of the N-rotor vehicle, USED FOR computing the
% FEED-FORWARD THRUST and CONTROLLER GAINS [kg]
nrotor_vehicle_mass_for_controller = 32e-3;

% The TRUE mass moment of inertia of the N-rotor vehicle, used for
% simulating the equations of motion [kg m^2]
nrotor_vehicle_inertia_true = [...
        16.57 ,   0.83 ,   0.72 ;...
         0.83 ,  16.66 ,   1.80 ;...
         0.72 ,   1.80 ,  29.26  ...
    ] * 1e-6;

% Inverse of the moment of inertia for convenience
nrotor_vehicle_inertia_true_inverse = inv(nrotor_vehicle_inertia_true);


% Specify a "baseline" value for the x and y coordinates of the propellers
% in the layout [meters]
x_baseline = (0.092/2) / sqrt(2);
y_baseline = (0.092/2) / sqrt(2);

% Specify a "baseline" value for the "torque-to-thrust" ratio to make
% constructing the "nrotor_vehicle_layout" matrix cleaner
c_baseline = 0.00596;
nrotor_vehicle_layout_true = [...
        [  1 , -1 , -1 ,  1 ] * x_baseline  ;...
        [ -1 , -1 ,  1 ,  1 ] * y_baseline  ;...
        [ -1 ,  1 , -1 ,  1 ] * c_baseline   ...
    ];

nrotor_vehicle_layout_for_controller = nrotor_vehicle_layout_true;
% SPECIFY THE MAXIMUM AND MINIMUM THRUST THAT CAN BE PRODUCED BY EACH
% PROPELLER:
% > This should be in unit of [Newtons]
% > For the minimum thrust: (this should always be zero)
nrotor_vehicle_thrust_min =       zeros( size(nrotor_vehicle_layout_true,2) , 1 );
% > For the maximum thrust:
nrotor_vehicle_thrust_max = 0.1597*ones( size(nrotor_vehicle_layout_true,2) , 1 );

exercise02_get_crazyflie_inner_controller;

% Sample time for the INNER loop controller
sample_time_controller_inner = 1/500;