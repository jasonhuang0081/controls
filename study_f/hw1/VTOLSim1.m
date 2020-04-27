clc;clear;
VTOLParam  % load parameters

% instantiate reference input classes 
reference = signalGenerator(0.5, 0.1);
thetaRef = signalGenerator(2*pi, 0.01);   
zRef = signalGenerator(0.5, 1);
hRef = signalGenerator(5, 0.5);
fRef = signalGenerator(5, 0.5);
tauRef = signalGenerator(2, 0.5);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.sin(t);
    theta = thetaRef.sin(t);
    z = zRef.sin(t);
    h = hRef.sin(t);
    f = fRef.sawtooth(t);
    tau = tauRef.sawtooth(t);
    % update animation and data plot
    state = [z; h; theta; 0.0; 0.0; 0.0];
    h_ref = 1;
    animation.update(state, r);
    dataPlot.update(t, state, r, h_ref, f, tau);
    t = t + P.t_plot;  % advance time by t_plot
    pause(0.1)
end


