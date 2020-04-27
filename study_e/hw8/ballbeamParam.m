% Ballbeam Parameter File

% Physical parameters of the ballbeam known to the controller
P.m1 = 0.35;    % kg
P.m2 = 2;       % kg
P.length = 0.5; % m
P.g = 9.81;     % m/s^2

% parameters for animation
P.radius = 0.05;   % radius of ball

% Initial Conditions
P.z0 = P.length/2;         % initial ball position, m
P.theta0 = 0.0*pi/180;     % initial beam angle, rads
P.zdot0 = 0.0;             % initial ball velocity, m/s
P.thetadot0 = 0.0;         % initial beam angular velocity, rads/s

% random uncertainty percentage added to the coefficients in the dynamics
P.alpha = 0.2;

% Simulation parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 50.0;   % End time of simulation
P.Ts = 0.01;      % sample time for controller
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% equilibrium force
P.Fe = P.m1*P.g*P.z0/P.length + P.m2*P.g/2;

% limits on force
P.Fmax = 15; % N

% kp and kd for inner loop
% P.kptheta = 182.5 * 2;   % use 0.1 s rise time, instead of 1
% P.kdtheta = 11.727 * 1.5;
% P.kitheta = 0.7;
P.kptheta = 126.741898;
P.kdtheta = 9.775257;
P.kitheta = 0;

% kp and kd for outer loop
% P.kpz = -0.4934;
% P.kdz = -0.317;
% P.kiz = 0.0;
P.kpz = -0.342621;
P.kdz = -0.264254;
P.kiz = -0.1;