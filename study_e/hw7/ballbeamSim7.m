ballbeamParam_hw7;  % load parameters

% instantiate pendulum, and reference input classes 
ballbeam = ballbeamDynamics(P);  
reference = signalGenerator(0.15, 0.05, 0.25); %magnitude 0.25 ± 0.15, frequency 0.01 Hz
controller = ballbeamController_hw7(P);
disturbance = signalGenerator(0, 0.0);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
%         r = step_ref.step(t);
        d = disturbance.step(t);  % compute disturbance
        z = ballbeam.state;
        u = controller.update(r,z);  % Calculate the input force
        y = ballbeam.update(u + d);  % Propagate/integrate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(ballbeam.state);
    dataPlot.update(t, r, ballbeam.state, u);
end
