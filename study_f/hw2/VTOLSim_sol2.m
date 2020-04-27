VTOLParam;  % load parameters

% instantiate VTOL, and reference input classes 
VTOL = VTOLDynamics(P);  
force = signalGenerator(10, 1);
torque = signalGenerator(-0.01, 2*pi*0.001);
% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        z_ref = 0;
        h_ref = 0;
        f = P.Fe + force.sin(t);  % Calculate the input force
        tau = torque.sin(t); % Input torque
        u = P.mixing*[f; tau];
        VTOL.update(u);  % Propagate/integrate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(VTOL.state, z_ref);
    dataPlot.update(t, VTOL.state, z_ref, h_ref, f, tau);
end
