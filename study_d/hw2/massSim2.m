massParam  % load parameters

% instantiate reference input classes 
massSystem = massDynamics_hw(P);
reference = signalGenerator(0.5, 0.02);
force = signalGenerator(2, 0.5);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = massAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f = force.sin(t);  % Calculate the input force
        massSystem.update(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
   
    animation.update(massSystem.state);
    dataPlot.update(t, r, massSystem.state, f);
%     pause(0.1)
end


