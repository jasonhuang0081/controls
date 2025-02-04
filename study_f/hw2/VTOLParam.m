% VTOL Parameter File
clear all

% Physical parameters of the VTOL known to the controller
P.mc = 1.0;  % kg
P.mr = 0.25;     % kg
P.Jc = 0.0042; %kg m^2
P.d = 0.3; % m
P.mu = 0.1; % kg/s
P.g = 9.81; % m/s^2
P.F_wind = 0;%0.1; % wind disturbance force

% uncertainty percentage of the physical VTOL parameters
P.alpha = 0.0;

% drawing parameters
P.length = 10;

% Initial Conditions
P.z0 = 0.0;        % initial lateral position
P.h0 = 0.0;          % initial altitude
P.theta0 = 0;      % initial roll angle
P.zdot0 = 0;       % initial lateral velocity
P.hdot0 = 0;       % initial climb rate
P.thetadot0 = 0;   % initial roll rate
P.target0 = 0;

% Simulation parameters
P.t_start = 0.0;  % Start time of simulation
P.t_end = 150.0;   % End time of simulation
P.Ts = 0.01;      % sample time for controller
P.t_plot = 0.1;   % the plotting and animation is updated at this rate

% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% equilibrium force
P.Fe = ((P.mc+2*P.mr)*P.g);

% mixing matrix
P.mixing = inv([1, 1; P.d, -P.d]);

% maximum force on each rotor
P.fmax = 10; % N

% kd and kp
% P.kpz = -0.00771 * 2;
% P.kdz = -0.0329;
% P.kiz = 0;
P.kpz = -0.054819;
P.kdz = -0.098906;
P.kiz = 0;
% P.kptheta = 0.372 *2;
% P.kdtheta = 0.191;
% P.kitheta = 0.5;
P.kptheta = 2.645867;
P.kdtheta = 0.510171;
P.kitheta = 0;
% P.kph = 0.1134 * 2;
% P.kdh = 0.5835;
% P.kih = 0.5;
P.kph = 0.806667;
P.kdh = 1.5554;
P.kih = 0.2;
