

flag_abs = true;
    
options = get_bode_options(flag_abs);

% defining the inner loop plant for lateral dynamics
P_lat_in = tf(1/(P.Jc+2*P.mr*P.d^2), [1,0,0]);
Plant = P_lat_in;
figure(5), clf
    bode(Plant,logspace(-3,5), options);
    hold on
    grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% only requirements are that we have about 60 deg PM and crossover freq 
% around 10 rad/s, and a low-pass filter.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


  figure(5), bode(Plant,logspace(-3,5), options), grid on
  %print('../../../figures/hw_vtol_lat_in_compensator_design_1','-dpdf','-bestfit')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = tf(1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% phase lead: increase PM (stability)
    wmax = 10.0; % location of maximum frequency bump
    M    = 15; % separation between zero and pole
    C = add_control_lead(C, wmax, M); 
    C = add_control_proportional(C, 5);

    figure(5), margin(Plant*C),  % update plot
    %print('../../../figures/hw_vtol_lat_in_compensator_design_2','-dpdf','-bestfit')

    
% low pass filter: decrease gain at high frequency (noise)
     p = 200;
     C = add_control_lpf(C, p);
     figure(5), margin(Plant*C),  % update plot
    %print('../../../figures/hw_vtol_lat_in_compensator_design_3','-dpdf','-bestfit')

 % no prefilter:
 F = tf([1], [1]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = minreal((Plant*C/(1+Plant*C)));
% closed loop transfer function from R to U
  CLOSED_R_to_U = minreal((C/(1+C*Plant)));

% update figure 2
figure(5), margin(Plant*C), 
grid on
    
figure(6), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y, options), hold on
        bodemag(CLOSED_R_to_Y*F, options)
        title('Closed Loop Bode Plot'), grid on
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
    %print('../../../figures/hw_vtol_lat_in_compensator_design_4','-dpdf','-bestfit')
       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[P.num_Clat_in,P.den_Clat_in] = tfdata(C,'v');

C_lat_in = C;

%--- general tracking specification ---
% track references above omega_r by gamma_r
function add_spec_tracking(gamma_r, omega_r, flag_abs)
    w = logspace(log10(omega_r)-2,log10(omega_r));

    if flag_abs
        plot(w,1/gamma_r*ones(size(w)),'g')
    else
        plot(w,20*log10(1/gamma_r)*ones(size(w)),'g')
    end
end

%--- steady state tracking of step ---
% track step to within gamma_r
function add_spec_tracking_step(gamma_r, flag_abs)
    w = logspace(-5, 0);
    if flag_abs
        plot(w, (1/gamma_r -1)*ones(size(w)), 'g');
    else
        plot(w, 20*log10(1/gamma_r -1)*ones(size(w)),'g')
    end
end

%--- steady state tracking of ramp ---
% track ramp to within gamma_r
function add_spec_tracking_ramp(gamma_r, flag_abs)
    w = logspace(-4, 0);
    if flag_abs
        plot(w,(1/gamma_r)./(w),'g')
    else
        plot(w,20*log10(1/gamma_r)-20*log10(w),'g')
    end
    
end

%--- steady state tracking of parabola ---
% track ramp to within gamma_r
function add_spec_tracking_parabola(gamma_r, flag_abs)
    w = logspace(-5, 0);
    if flag_abs
        plot(w,(1/gamma_r)./(w.^2),'g')
    else
        plot(w,20*log10(1/gamma_r)-40*log10(w),'g')
    end
end

%--- input disturbance specification ---
% reject distubance below omega_d by gamma_d
function add_spec_disturbance(gamma_d, omega_d, Plant, flag_abs, options)
    if flag_abs
        w = logspace(log10(omega_d)-2, log10(omega_d));
        Pmag=bode(Plant,w, options);
        for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
        plot(w,1/gamma_d*ones(1,length(Pmag_)).*Pmag_,'g')     
    else
        w = logspace(log10(omega_d)-2, log10(omega_d));
        Pmag=bode(Plant,w, options);
        for i=1:size(Pmag,3), Pmag_(i)=Pmag(1,1,i); end
        plot(w,20*log10(1/gamma_d)*ones(1,length(Pmag_))+20*log10(Pmag_),'g')
    end
end

%--- noise specification ---
% attenuate noise about omega_n by gamma_n
function add_spec_noise(gamma_n, omega_n, flag_abs)
	w = logspace(log10(omega_n),2+log10(omega_n));
    if flag_abs
    	plot(w,gamma_n*ones(size(w)),'g')
    else
        plot(w,20*log10(gamma_n)*ones(size(w)),'g')
    end
end

% proportional control: change cross over frequency
% proportional gain kp
function Cnew = add_control_proportional(C, kp)
	Cnew = C*kp;
end

% integral control: increase steady state tracking and dist rejection
% ki: frequency at which integral action ends
function Cnew = add_control_integral(C, ki) 
	Integrator = tf([1, ki], [1, 0]);
	Cnew = C*Integrator;
end

% phase lag: add gain at low frequency (tracking, dist rejection)
% z: frequency at which lag ends
% M: separation between pole and zero
function Cnew = add_control_lag(C, z, M)
    Lag = tf([1, z], [1, z/M]);
	Cnew = C * Lag;
end

% low pass filter: decrease gain at high frequency (noise)
% p:  low pass filter frequency
function Cnew = add_control_lpf(C, p)
    LPF = tf(p,[1, p]);
    Cnew = C * LPF;
end

% phase lead: increase PM (stability)
% wmax: location of maximum frequency bump
% M: separation between zero and pole
function Cnew = add_control_lead(C, omega_L, M)
    gain = (1+sqrt(M))/(1+1/sqrt(M));
    Lead =tf(gain * [1, omega_L/sqrt(M)], [1, omega_L*sqrt(M)]);
	Cnew = C * Lead;
end

% notch filter: reduce closed loop peaking at cross over
% ws: frequency to start the notch
% M: width of the notch
function Cnew = add_control_notch(C, ws, M)
    Notch = tf([1,2*sqrt(M)*ws,M*ws^2],[1,(M+1)*ws,M*ws^2]);
	Cnew = Notch * C;
end


function options = get_bode_options(flag)
    options = bodeoptions;
    options.Title.FontSize = 14;
    options.Title.FontWeight = 'Bold';
    options.XLabel.FontSize = 14;
    options.XLabel.FontWeight = 'Bold';
    options.YLabel.FontSize = 14;
    options.YLabel.FontWeight = 'Bold';
    options.TickLabel.FontSize = 11;
    options.Grid = 'on';
    
    if flag == true;
        options.MagUnits = 'abs';
        options.MagScale = 'log';

    end
end





