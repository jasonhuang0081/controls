ballbeamParam;  % load parameters

% instantiate pendulum, and reference input classes 
ballbeam = ballbeamDynamics(P);  
force = signalGenerator(11, 2*pi*0.05, 10.0);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = 0;
        u = force.sin(t);  % Calculate the input force
        y = ballbeam.update(u);  % Propagate/integrate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(ballbeam.state);
    dataPlot.update(t, r, ballbeam.state, u);
end
