VTOLParam  % load parameters

% instantiate reference input classes 
VTOL = VTOLDynamics_hw(P);
reference = signalGenerator(0.5, 0.02);
force = signalGenerator(7, 1, 7);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = VTOLAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f1 = force.sin(t);  
        f2 = force.sin(t);
        u = [f1;f2];
        VTOL.update(u);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
%     if VTOL.state(2) < 0
%         VTOL.state(2) = 0;
%     end
    animation.update(VTOL.state, r);
    dataPlot.update(t, VTOL.state,r,r, f1,f1);
%     pause(0.1)
end


