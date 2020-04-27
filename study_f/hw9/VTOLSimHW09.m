clear path
VTOLParamHW09;  % load parameters

% instantiate VTOL, controller, and reference input classes 
% Instantiate Dynamics class
VTOL = VTOLDynamics(P);  
controller = VTOLControllerHW09(P); 
z_reference = signalGenerator(2.5, 0.02, 3);  
h_reference = signalGenerator(3, 0.03, 5); 
z_noise = signalGenerator(0.001);
h_noise = signalGenerator(0.001);
th_noise = signalGenerator(0.001);

%these are pretty significant disturbances acting as a square wave, much
%simpler if they are only step waves as shown below.
F_disturbance = signalGenerator(0.25, 0.1);
tau_disturbance = signalGenerator(0.25, 0.1);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);
dataPlotObserver = dataPlotterObserverHW09(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = VTOL.h();
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        h_ref = h_reference.square(t);
        z_ref = z_reference.square(t);
        d_F = F_disturbance.square(t);  %can call these as ".step(t)" instead for a single input disturbance
        d_tau = tau_disturbance.square(t); %can call these as ".step(t)" instead
%         n = [0; 0; 0];  % sensor noise
        n = [z_noise.random(t); h_noise.random(t); th_noise.random(t)]; %noise 
        u = controller.update([z_ref; h_ref], y + n);  
        y = VTOL.update(P.mixing * (u + [d_F; d_tau])); 
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(VTOL.state, 0.0);
    dataPlot.update(t, VTOL.state, z_ref, h_ref, u(1), u(2));
    dataPlotObserver.update(t, VTOL.state, controller.xhat_lon, controller.xhat_lat);

end


