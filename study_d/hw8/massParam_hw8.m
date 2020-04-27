% mass - parameter file
massParam % general parameters

% tuning parameters
%tr = 2;         % rise time from D.8
tr = 2.0;       % slower rise time to avoid saturation
zeta = 0.9;
integrator_pole = -10;

P.alpha = 0.2;

% state space design
A1 = [...
    0, 1;...
    -P.k/P.m, -P.b/P.m;...
    ];
B1 = [0; 1/P.m ];
C = [...
    1, 0;...
    ];
A = [A1,zeros(2,1); -C,0];
B = [B1; 0];

% gain calculation
wn = 2.2/tr;
ol_char_poly = charpoly(A);
des_char_poly = conv([1,2*zeta*wn,wn^2],poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A,B))~=3, disp('System Not Controllable'); end
K1 = place(A,B,des_poles);
P.K  = K1(1:2);
P.ki = K1(3);


fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
