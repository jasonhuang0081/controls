massParam;  % load parameters

% instantiate mass, and reference input classes 
mass = massDynamics(P);  
force = signalGenerator(10, 0.05);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = massAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = 0;
        f = force.sin(t);  % Calculate the input force
        y = mass.update(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(mass.state);
    dataPlot.update(t, r, mass.state, f);
end
