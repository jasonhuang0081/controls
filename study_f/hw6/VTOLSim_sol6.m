clear path
VTOLParamHW06;  % load parameters

% instantiate VTOL, controller, and reference input classes 
VTOL = VTOLDynamics(P);  
controller = VTOLControllerHW06(P);  
z_reference = signalGenerator(2.5, 0.02, 3);  
h_reference = signalGenerator(3, 0.03, 5); 
F_disturbance = signalGenerator(0.5, 0);
tau_disturbance = signalGenerator(0, 0);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
y = VTOL.h();
while t < P.t_end  
    % Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        h_ref = h_reference.square(t);
        z_ref = z_reference.square(t);
        d_F = F_disturbance.step(t);
        d_tau = tau_disturbance.step(t);
        n = [0; 0; 0];  % sensor noise
        u = controller.update([z_ref; h_ref], y + n);  
        y = VTOL.update(P.mixing * (u + [d_F; d_tau])); 
        t = t + P.Ts; % advance time by Ts
    end
    % update animation and data plots
    animation.update(VTOL.state, 0.0);
    dataPlot.update(t, VTOL.state, z_ref, h_ref, u(1), u(2));
end
