% inverted ballbeam - parameter file for hw8
VTOLParam % general ballbeam parameters

% tuning parameters
tr_h    = 3;        % rise time from F.8
% tr_h    = tr_h/4;   % faster!
wn_h    = 2.2/tr_h;
zeta_h  = 0.707;
integrator_pole_h = -5;

tr_z    = 3;        % rise time from F.8
% tr_z    = tr_z/4;   % faster!
wn_z    = 2.2/tr_z;
zeta_z  = 0.707;
integrator_pole_z = -5;

tr_th   = tr_z/10;
wn_th   = 2.2/tr_th;
zeta_th = 0.707;


% equilibrium force and constraints
P.Fe = (P.mc+2*P.mr)*P.g;
P.Ftildemax = 2*P.fmax - P.Fe;
P.taumax = (P.fmax-P.Fe/2)/P.d;

% state space design
A_lon1 = [...
    0, 1;...
    0, 0;...
    ];
B_lon1 = [0; 1/(P.mc+2*P.mr)];
C_lon = [1, 0];
A_lon = [A_lon1, zeros(2,1);-C_lon,0];
B_lon = [B_lon1;0];

A_lat1 = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -(P.Fe/(P.mc+2*P.mr)), -(P.mu/(P.mc+2*P.mr)), 0;...
    0, 0, 0, 0;...
    ];
B_lat1 = [0;0;0;1/(P.Jc+2*P.mr*P.d^2)];
C_lat = [1, 0, 0, 0; 0, 1, 0, 0];
Cr_lat = [1, 0, 0, 0];
A_lat = [A_lat1, zeros(4,1);-Cr_lat,0];
B_lat = [B_lat1;0];

% gain calculation
% ol_char_poly_lon = charpoly(A_lon);
des_char_poly_lon = conv([1,2*zeta_h*wn_h,wn_h^2],poly(integrator_pole_h));
des_poles_lon = roots(des_char_poly_lon);

% ol_char_poly_lat = charpoly(A_lat);
des_char_poly_lat = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]),poly(integrator_pole_z));
des_poles_lat = roots(des_char_poly_lat);

% gains for longitudinal system
if rank(ctrb(A_lon,B_lon))~=3 
	disp('Lon System Not Controllable');
end
K1 = place(A_lon,B_lon,des_poles_lon);
P.K_lon = K1(1:2);
P.ki_lon = K1(3);

% gains for lateral system
if rank(ctrb(A_lat,B_lat))~=5
	disp('Lat System Not Controllable'); 
end
K1 = place(A_lat,B_lat,des_poles_lat);
P.K_lat = K1(1:4);
P.ki_lat = K1(5);


sprintf('K_lat: (%f, %f, %f, %f)\nki_lat: %f\nK_lon: (%f, %f)\nki_lon: %f\n',...
    P.K_lat(1), P.K_lat(2), P.K_lat(3), P.K_lat(4), P.ki_lat(1),...
    P.K_lon(1), P.K_lon(2), P.ki_lon(1))

