% inverted ballbeam - parameter file for hw8
ballbeamParam % general ballbeam parameters

P.alpha = 0.2; 

% dirty derivative parameters
P.sigma = 0.005; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% saturation limit for beam angle
P.theta_max = 30.0*pi/180.0;  % Max theta, rads

% equilibrium position
P.ze = P.length/2;

% tuning parameters
integrator_pole = -5;
tr_z = 1.2; % tuned for fastest rise time without saturation
tr_th = 0.5; % rise time for angle
% tr_z = 10;  % rise time for z from E.8
% tr_th = 1;  % rise time for theta from E.8

zeta_z = 0.707; % damping ratio for outer loop
zeta_th  = 0.707; % damping ratio for inner loop

%[z, theta, z_dot, theta_dot]
% state space design
A1 = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -P.g, 0, 0;...
    -P.m1*P.g/((P.m2*P.length^2)/3+P.m1*(P.length/2)^2), 0, 0, 0;
...
];
B1 = [0; 0; 0; P.length/(P.m2*P.length^2/3+P.m1*P.length^2/4) ];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];
Cr = [1,0,0,0];
A = [A1,zeros(4,1);-Cr,zeros(1,1)];
B = [B1;0];
% compute gains
wn_z     = 2.2/tr_z;
wn_th    = 2.2/tr_th; 
% ol_char_poly = charpoly(A);
des_char_poly = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]), poly(integrator_pole));
des_poles = roots(des_char_poly);
% Compute the gains if the system is controllable
if rank(ctrb(A, B)) ~= 5
    disp('The system is not controllable')
end
K1 = place(A, B, des_poles);
P.K  = K1(1:4);
P.ki = K1(5);

fprintf('\t K: [%f, %f, %f, %f]\n', P.K(1), P.K(2), P.K(3), P.K(4))
fprintf('\t ki: %f\n', P.ki)
