% load controller design
if 1
    loopshape_massHW11
end

% instantiate mass, controller, and reference input classes 
% Instantiate Dynamics class
mass = massDynamics(P);  
ctrl = massControllerHW11(P);  
amplitude = 0.5; % amplitude of reference input
frequency = 0.04; % frequency of reference input
reference = signalGenerator(amplitude, frequency);  

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = massAnimation(P);

% set disturbance input
disturbance = 0.25;

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Get referenced inputs from signal generators
    ref_input = reference.square(t);
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        u = ctrl.update(ref_input, mass.h());  % Calculate the control value
        sys_input = u+disturbance;  % input to plant is control input + disturbance
        mass.update(sys_input);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(mass.state);
    dataPlot.update(t, ref_input, mass.state, u);
end


