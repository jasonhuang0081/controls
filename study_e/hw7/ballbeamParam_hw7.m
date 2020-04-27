% inverted ballbeam - parameter file for hw8
ballbeamParam % general ballbeam parameters

% setting uncertainty
P.alpha = 0.0; 

% dirty derivative parameters
P.sigma = 0.005; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% saturation limit for beam angle
P.theta_max = 30.0*pi/180.0;  % Max theta, rads

% tunning parameters
tr_th = 0.5;  % tuned value of the rise time
zeta_th = 0.707;
zeta_z = 0.707;
tr_z = 1.2;
wn_th = 2.2/tr_th;
wn_z = 2.2/tr_z;


A = [0 0 1 0;
    0 0 0 1;
    0 -P.g 0 0;
    -P.m1*P.g/((P.m2*P.length^2)/3 + P.m1*P.z0^2) 0 0 0];
B = [0; 0; 0; P.length/((P.m2*P.length^2)/3 + P.m1*P.z0^2)];
% C = [1 0 0 0;
%     0 1 0 0];
C = [1 0 0 0];

a = 2*zeta_th*wn_th;
b = wn_th^2;
c = 2*zeta_z*wn_z;
d = wn_z^2;
des_char_poly = [1,a+c,b+a*c+d,b*c+a*d,b*d];
des_poles = roots(des_char_poly);

if rank(ctrb(A,B))~=4 
    disp('System Not Controllable'); 
else
    P.K = place(A,B,des_poles); 
    P.kr = -1/(C*inv(A-B*P.K)*B);
end

fprintf('\t K: [%f, %f]\n', P.K)
fprintf('\t kr: %f\n', P.kr)


