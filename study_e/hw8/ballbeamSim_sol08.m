ballbeamParamHW08;  % load parameters

% instantiate ballbeam, controller, and reference input classes 
ballbeam = ballbeamDynamics(P);  
controller = ballbeamControllerHW08(P);  
reference = signalGenerator(0.125, 0.02, 0.25);  
disturbance = signalGenerator(0.25, 0.1);  

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
        d = disturbance.step(t);            %%%%% can change to square to see oscilate wave
        n = [0; 0];  % sensor noise
        u = controller.update(r, y + n);  % Calculate the control value
        y = ballbeam.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(ballbeam.state);
    dataPlot.update(t, r, ballbeam.state, u);
end