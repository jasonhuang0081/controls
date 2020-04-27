massParam;  % load parameters

% instantiate mass, and reference input classes 
mass = massDynamics(P);  
reference = signalGenerator(0.5, 0.02);
disturbance = signalGenerator(0.25, 0.0);
controller = massController_hw6(P);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = massAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);  % compute disturbance
        z = mass.state;
        f = controller.update(r,z,P.Ts);  % Calculate the input force
        y = mass.update(f + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(mass.state);
    dataPlot.update(t, r, mass.state, f);
end
