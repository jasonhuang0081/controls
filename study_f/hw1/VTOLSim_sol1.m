VTOLParam  % load parameters

% instantiate reference input classes 
zSimData = signalGenerator(0.5, 0.1, 0.5);
hSimData = signalGenerator(0.5, 0.1, 0.5);
thetaSimData = signalGenerator(pi/4, 0.05*2*pi, 0.1); 

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start

while t < P.t_end  
    % set variables
    z_r = 0;
    h_r = 0;
    tau = 0; 
    f = 0;
    
    z = zSimData.sin(t);
    h = hSimData.sin(t);
    theta = thetaSimData.sin(t);
    
    

    % update animation and data plot
    state = [z; h; theta; 0.0; 0.0; 0.0];
    animation.update(state, z_r);
    dataPlot.update(t, state, z_r, h_r, f, tau);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.05)
end


