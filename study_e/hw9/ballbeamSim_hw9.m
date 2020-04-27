ballbeamParam_hw9;  % load parameters

% instantiate ballbeam, controller, and reference input classes 
ballbeam = ballbeamDynamics(P);  
controller = ballbeamController_hw9(P);  
reference = signalGenerator(0.125, 0.02, 0.25);  
disturbance = signalGenerator(0.1, 0.1);  
noise_z = signalGenerator(0.001);
noise_th = signalGenerator(0.001);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);
obsplot = dataPlotterObserver_hw9(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = ballbeam.h();  % output at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        r = reference.square(t);
        d = disturbance.step(t);
%         d = 0;
        n = [noise_z.random(t); noise_th.random(t)]; 
        [u,xhat] = controller.update(r, y+n);  % Calculate the control value
        y = ballbeam.update(u + d);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(ballbeam.state);
    dataPlot.update(t, r, ballbeam.state, u);
    obsplot.update(t,ballbeam.state, xhat);
end


