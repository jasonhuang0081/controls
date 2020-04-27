massParam

%getting parameters from HW06, C.10
massParamHW06


flag_abs = true;

options = get_bode_options(flag_abs);


% transfer function for robot arm
Plant = tf([1/P.m],[1, P.b/P.m, P.k/P.m]);

figure(2), clf
bodemag(Plant,logspace(-4,4), options)
hold on
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Define Design Specifications
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--- general tracking specification ---
omega_r = 0.1;  % track signals below this frequency
gamma_r = 0.03;  % tracking error below this value
add_spec_tracking(gamma_r, omega_r, flag_abs);
        
%--- noise specification ---
omega_n = 500;  % attenuate noise above this frequency
gamma_n = 0.001;   % attenuate noise by this amount
add_spec_noise(gamma_n, omega_n, flag_abs);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control Design
  C = tf(1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% integral control: increase steady state tracking and dist rejection
     k_I = 0.2; % frequency at which integral action ends
     C = add_control_integral(C, k_I);
     figure(2), margin(Plant*C),  % update plot
     
% phase lead: increase PM (stability)
w_max = 7; % location of maximum frequency bump
phi_max = 60*pi/180;
M = (1+sin(phi_max))/(1-sin(phi_max)); % lead ratio
C = add_control_lead(C, w_max, M)
figure(2), margin(Plant*C),  % update plot

% find gain to set crossover at w_max = 7 rad/s
[m,p] = bode(C*Plant,w_max);
K = 1/m;
C = add_control_proportional(C, K);
figure(2), margin(C*Plant), grid on,  % update plot
legend('plant alone','tracking spec', 'noise spec', 'PI','PI/lead, K=1',strcat('PI/lead, K=',num2str(K,3)));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Prefilter Design
% low pass filter
  p = 1;  % frequency to start the LPF
  F = tf(1,1);
  F = add_control_lpf(F, p)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % prefilter: reduce closed loop peaking at cross over
%     % notch filter
%     wnotch = 40;  % center frequency of notch
%     D = 5; % depth of notch
%     M  = 20;  % width of the notch
%     NOTCH = tf([1,(D/sqrt(M)+sqrt(M)/D)*wnotch,wnotch^2],...
%                [1,(1/sqrt(M)+sqrt(M))*wnotch,wnotch^2]);
%     F = F*NOTCH;
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create plots for analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Open-loop tranfer function 
  OPEN = Plant*C;
% closed loop transfer function from R to Y
  CLOSED_R_to_Y = minreal((Plant*C/(1+Plant*C)));
% closed loop transfer function from R to U
  CLOSED_R_to_U = minreal((C/(1+C*Plant)));
    
figure(3), clf
    subplot(3,1,1), 
        bodemag(CLOSED_R_to_Y), hold on
        bodemag(CLOSED_R_to_Y*F)
        title('Closed Loop Bode Plot'), grid on
    subplot(3,1,2), 
        step(CLOSED_R_to_Y), hold on
        step(CLOSED_R_to_Y*F)
        title('Closed loop step response'), grid on
    subplot(3,1,3), 
        step(CLOSED_R_to_U), hold on
        step(CLOSED_R_to_U*F)
        title('Control effort for step response'), grid on
%print('../../../figures/hw_mass_compensator_design_5','-dpdf','-bestfit')
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Convert controller to state space equations for implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[P.num_C,P.den_C] = tfdata(C,'v');
[P.num_F,P.den_F] = tfdata(F,'v');

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
    	plot(w,gamma_n*ones(size(w)),'g--')
    else
        plot(w,20*log10(gamma_n)*ones(size(w)),'g--')
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