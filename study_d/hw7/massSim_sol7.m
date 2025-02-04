massParamHW07;  % load parameters

% instantiate mass, controller, and reference input classes 
mass = massDynamics(P);  
controller = massControllerHW07(P);  
reference = signalGenerator(0.5, 0.02);  
disturbance = signalGenerator(0.0, 0);  

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = massAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = mass.h();  % output at start time
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);
        n = 0;  
        u = controller.update(r, y + n);  % Calculate the control value
        y = mass.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(mass.state);
    dataPlot.update(t, r, mass.state, u);
end
