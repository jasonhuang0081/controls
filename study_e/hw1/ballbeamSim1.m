ballbeamParam  % load parameters

% instantiate reference input classes 
reference = signalGenerator(0.5, 0.1);
zRef = signalGenerator(0.1, 0.05*pi);
thetaRef = signalGenerator(0.1, 0.5*pi);   
fRef = signalGenerator(5, 0.5);


% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    z = zRef.sin(t);
    theta = thetaRef.sin(t);
    f = fRef.sawtooth(t);
    % update animation and data plot
    state = [z; theta; 0.0; 0.0];
    animation.update(state);
    dataPlot.update(t, r, state, f);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


