%  ---------------------------------------------------------------------  %
%    CCCC  RRRR     A    ZZZZZ  Y   Y  FFFFF  L      III  EEEEE
%   C      R   R   A A      Z    Y Y   F      L       I   E
%   C      RRRR   A   A    Z      Y    FFF    L       I   EEE
%   C      R   R  AAAAA   Z       Y    F      L       I   E
%    CCCC  R   R  A   A  ZZZZZ    Y    F      LLLLL  III  EEEEE
%
%   III  N   N  N   N  EEEEE  RRRR
%    I   NN  N  NN  N  E      R   R
%    I   N N N  N N N  EEE    RRRR
%    I   N  NN  N  NN  E      R   R
%   III  N   N  N   N  EEEEE  R   R
%
%    CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
%   C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
%   C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
%   C      O   O  N  NN    T    R  R   O   O  L      L      E      R  R
%    CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
%  ---------------------------------------------------------------------  %
%% LOAD THE PID PARAMTERS OF THE RATE CONTROLLER THAT RUN ON-BOARD THE CRAZYFLIE

% INFO:
%  > Parameters are taken directly from the crazyflie firmware,
%  > The derivative gains have been reduced by a factor of 10 for avoid
%    high-frequency switching of the control actuation.



pid_x_body_rate_kp = 250.0;
pid_x_body_rate_ki = 500.0;
pid_x_body_rate_kd =   2.5*0.1;
pid_x_body_rate_integrator_limit = 33.3;

pid_y_body_rate_kp = 250.0;
pid_y_body_rate_ki = 500.0;
pid_y_body_rate_kd =   2.5*0.1;
pid_y_body_rate_integrator_limit = 33.3;

pid_z_body_rate_kp = 120.0;
pid_z_body_rate_ki =  16.7;
pid_z_body_rate_kd =   0.0;
pid_z_body_rate_integrator_limit = 167.7;





%  ---------------------------------------------------------------------  %
%                      t     1    6           CCCC  M   M  DDDD
%   u   u  i    n nn  ttt   11   6           C      MM MM  D   D
%   u   u  i    nn  n  t     1   6666        C      M M M  D   D
%   u  uu  i    n   n  t     1   6   6       C      M   M  D   D
%    uu u  iii  n   n  ttt  111   666         CCCC  M   M  DDDD
%  ---------------------------------------------------------------------  %
%% SPECIFY THE CONVERSION FROM [uint16] COMMAND TO THRUST

% Specify the coefficients of the conversion from a uint16 binary command
% to the propeller thrust in Newtons
cmd_2_newtons_conversion_quadratic_coefficient  =  1.3385e-10;
cmd_2_newtons_conversion_linear_coefficient     =  6.4870e-6;

% Specify that max and min for the uint16 binary command
cmd_max = 2^16-1;
cmd_min = 0;





%% ASCII ART OF THE CRAZYFLIE 2.0 LAYOUT
%
%  > M1 to M4 stand for Motor 1 to Motor 4
%  > "CW"  indicates that the motor rotates Clockwise
%  > "CCW" indicates that the motor rotates Counter-Clockwise
%
%
%        ____                         ____
%       /    \                       /    \
%  (CW) | M4 |           x           | M1 | (CCW)
%       \____/\          ^          /\____/
%            \ \         |         / /
%             \ \        |        / /
%              \ \______ | ______/ /
%               \        |        /
%                |       |       |
%        y <-------------o       |
%                |               |
%               / _______________ \
%              / /               \ \
%             / /                 \ \
%        ____/ /                   \ \____
%       /    \/                     \/    \
% (CCW) | M3 |                       | M2 | (CW)
%       \____/                       \____/
%
%
%
%  ---------------------------------------------------------------------  %