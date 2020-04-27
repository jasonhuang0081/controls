% inverted ballbeam - parameter file for hw6
VTOLParam % general ballbeam parameters

% uncertainty parameter
P.alpha = 0.0; 

% tuning parameters
%tr_h = 8;  % rise time for altitude - original
tr_h = 3; % turned for fastest performance without saturation
zeta_h = 0.707; % damping ratio for altitude
%tr_z = 8; % rise time for outer lateral loop (position) - original
tr_z = 3;%3; % tuned for fastest performance without saturation
M = 10; % time separation between inner and outer lateral loops
tr_th = 3/M;
zeta_z = 0.707; % damping ratio for outer lateral loop
zeta_th = 0.707; % damping ratio for inner lateral loop
wn_th = 2.2 / tr_th;
wn_z = 2.2 / tr_z;
wn_h = 2.2 / tr_h;

% lateral theta
a = 2*zeta_th*wn_th;
b = wn_th^2;
% lateral z
c = 2*zeta_z*wn_z;
d = wn_z^2;
% vertial h
e = 2*zeta_h*wn_h;
f = wn_h^2;

des_char_poly = conv([1 a b],[1 c d]);
des_poles = roots(des_char_poly)

v1 = -P.Fe/(P.mc + 2*P.mr);
v2 = -P.mu/(P.mc + 2*P.mr);
% augment bottom right to be vertical control and top left to be horizontal
% A = [0  0  1  0 0 0;    %z
%      0  0  0  1 0 0;    %theta
%      0  v1 v2 0 0 0;    %zdot
%      0  0  0  0 0 0;    %thetadot
%      0  0  0  0 0 1;    %h
%      0  0  0  0 0 0];   %hdot
 A = [0  0  1  0 ;    %z
     0  0  0  1 ;   %theta
     0  v1 v2 0 ;
     0  0  0  0];
% B = [0; 0; 0; 1/(P.Jc+2*P.mr*P.d^2); 0; 1/(P.mc+2*P.mr)];
B = [0; 0; 0; 1/(P.Jc+2*P.mr*P.d^2)];
C = [1 0 0 0];

if rank(ctrb(A,B))~=4
    disp('System lateral Not Controllable'); 
else
    P.K_lat = place(A,B,des_poles); 
    P.kr_lat = -1/(C*inv(A-B*P.K_lat)*B);
end

des_char_poly = [1 e f];
des_poles = roots(des_char_poly)
A = [0 1;
    0 0 ];
B = [0; 1/(P.mc+2*P.mr)];
C = [1 0];

if rank(ctrb(A,B))~=2
    disp('System vertical Not Controllable'); 
else
    P.K_ver = place(A,B,des_poles); 
    P.kr_ver = -1/(C*inv(A-B*P.K_ver)*B);
end
fprintf('\t K laterial: [%f, %f]\n', P.K_lat)
fprintf('\t kr laterial: %f\n', P.kr_lat)
fprintf('\t K vertical: [%f, %f]\n', P.K_ver)
fprintf('\t kr vertical: %f\n', P.kr_ver)

% dirty derivative gain for differentiator
P.sigma = 0.005;

% saturation limits for linearized control
P.Ftildemax = 2*P.fmax - P.Fe;
P.taumax = (P.fmax-P.Fe/2)/P.d;





