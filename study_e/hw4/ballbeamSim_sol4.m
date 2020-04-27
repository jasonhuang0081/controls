%restoredefaultpath
ballbeamParamHW04;  % load parameters

% instantiate ballbeam, controller, and reference input classes 
ballbeam = ballbeamDynamics(P);  
controller = ballbeamController(P);  
reference = signalGenerator(0.15, 0.05, 0.25);  
disturbance = signalGenerator(0.0, 0);  

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = ballbeam.h();  % output at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = reference.step(t);
        n = [0; 0];  % sensor noise
        x = ballbeam.state;
        u = controller.update(r, x);  % Calculate the control value
        y = ballbeam.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(ballbeam.state);
    dataPlot.update(t, r, ballbeam.state, u);
end


