VTOLParam;  % load parameters

% instantiate VTOL, and reference input classes 
VTOL = VTOLDynamics(P);  
reference_z = signalGenerator(2.5, 0.02, 3);
reference_h = signalGenerator(3, 0.03, 5); 
controller = VTOLController_hw6(P);
disturbance = signalGenerator(0.5, 0.0);
% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        z_ref = reference_z.square(t);
        h_ref = reference_h.square(t);
        d = disturbance.step(t);  % compute disturbance
        z = VTOL.state;
        [u, f, tau] = controller.update(z_ref,h_ref,z,P.Ts);
        y = VTOL.update(u + d);  % Propagate/integrate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(VTOL.state, z_ref);
%     dataPlot.update(t, VTOL.state, z_ref, h_ref, f, tau);
    dataPlot.update(t, VTOL.state, z_ref, h_ref, u(1), u(2));
end
