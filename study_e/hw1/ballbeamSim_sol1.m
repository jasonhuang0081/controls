ballbeamParam  % load parameters

% instantiate reference input classes 
zSimData = signalGenerator(0.5, 0.05*pi);
thetaSimData = signalGenerator(0.1, 0.5*pi);   


% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start

while t < P.t_end  
    % set variables
    r = 0;
    f = 0;
    theta = thetaSimData.sin(t);

    z = zSimData.sin(t);  
    %for a real system, this may not be necessary, but this forces the ball
    %to not go in the negative direction. 
    if z <0
        z = 0;
    end
    
    % update animation and data plot
    state = [z; theta; 0.0; 0.0];
    animation.update(state);
    dataPlot.update(t, r, state, f);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.05)
end


