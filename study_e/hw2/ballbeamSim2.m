ballbeamParam  % load parameters

% instantiate reference input classes 
ballbeam = ballbeamDynamics_hw(P);
reference = signalGenerator(0.5, 0.02);
force = signalGenerator(20, 1, 2);

% instantiate the data plots and animation
dataPlot = dataPlotter(P);
animation = ballbeamAnimation(P);

% main simulation loop
t = P.t_start;  % time starts at t_start
while t < P.t_end  
    % set variables
    r = reference.square(t);
    t_next_plot = t + P.t_plot;
    while t < t_next_plot % updates control and dynamics at faster simulation rate
        f = force.sin(t);  % Calculate the input force
        ballbeam.update(f);  % Propagate the dynamics
        t = t + P.Ts; % advance time by Ts
    end
%     if ballbeam.state(1) < 0
%         ballbeam.state(1) = 0;
%     end
%     if ballbeam.state(1) > 0
%         ballbeam.state(1) = 0.5;
%     end
%     if ballbeam.state(2) < 0
%         ballbeam.state(2) = 0;
%     end
%     if ballbeam.state(2) > pi/2
%         ballbeam.state(2) = pi/2;
%     end

    animation.update(ballbeam.state);
    dataPlot.update(t, r, ballbeam.state, f);
%     pause(0.1)
end


