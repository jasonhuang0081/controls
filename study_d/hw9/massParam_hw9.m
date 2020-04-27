% mass - parameter file
massParam % general parameters

% tuning parameters
%tr = 2;         % rise time from D.8
tr = 2.0;       % slower rise time to avoid saturation
zeta = 0.9;
integrator_pole = -10;

P.alpha = 0.2;

% state space design
P.A1 = [...
    0, 1;...
    -P.k/P.m, -P.b/P.m;...
    ];
P.B1 = [0; 1/P.m ];
P.C = [...
    1, 0;...
    ];
A = [P.A1,zeros(2,1); -P.C,0];
B = [P.B1; 0];

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
    
wn_o = 10*wn;
des_obsv_char_poly = [1,2*zeta*wn_o,wn_o^2];
des_obsv_poles = roots(des_obsv_char_poly);

if rank(obsv(P.A1,P.C))~=2
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A1',P.C',des_obsv_poles)'; 
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L^T: [%f, %f]\n', P.L(1), P.L(2))