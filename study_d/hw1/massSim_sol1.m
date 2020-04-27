massParam  % load parameters

% instantiate reference input classes 
zSimData = signalGenerator(1, 0.1);   

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = massAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = 0;
    f = 0;
    z = zSimData.sin(t);

    
    % update animation and data plot
    state = [z; 0.0];
    animation.update(state);
    dataPlot.update(t, r, state, f);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


