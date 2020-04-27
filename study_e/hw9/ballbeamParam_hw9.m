% inverted ballbeam - parameter file for hw8
ballbeamParam % general ballbeam parameters

P.alpha = 0.2; 

% dirty derivative parameters
P.sigma = 0.005; % P.Cutoff freq for dirty derivative
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
% state spaP.Ce design
P.A1 = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -P.g, 0, 0;...
    -P.m1*P.g/((P.m2*P.length^2)/3+P.m1*(P.length/2)^2), 0, 0, 0;
...
];
P.B1 = [0; 0; 0; P.length/(P.m2*P.length^2/3+P.m1*P.length^2/4) ];
P.C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];
P.Cr = [1,0,0,0];
A = [P.A1,zeros(4,1);-P.Cr,zeros(1,1)];
B = [P.B1;0];
% P.Compute gains
wn_z     = 2.2/tr_z;
wn_th    = 2.2/tr_th; 
% ol_P.Char_poly = P.Charpoly(A);
des_Char_poly = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]), poly(integrator_pole));
des_poles = roots(des_Char_poly);
% P.Compute the gains if the system is P.Controllable
if rank(ctrb(A, B)) ~= 5
    disp('The system is not P.Controllable')
end
K1 = place(A, B, des_poles);
P.K  = K1(1:4);
P.ki = K1(5);

wn_o = 10*wn_th;
wn_o_z = 10*wn_z;
des_obsv_char_poly = conv([1,2*zeta_th*wn_o,wn_o^2],[1,2*zeta_z*wn_o_z,wn_o_z^2]);
des_obsv_poles = roots(des_obsv_char_poly);

if rank(obsv(P.A1,P.C))~=4
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A1',P.C',des_obsv_poles)'; 
end


fprintf('\t K: [%f, %f, %f, %f]\n', P.K(1), P.K(2), P.K(3), P.K(4))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L: [%f, %f, %f, %f\n %f, %f, %f, %f]\n', P.L(1,1), P.L(2,1), P.L(3,1), P.L(4,1),P.L(1,2), P.L(2,2), P.L(3,2), P.L(4,2))