% mass - parameter file
massParam % general parameters

% tuning parameters
tr = 2.0;  % tuned value of the rise time
zeta = 0.9;
integrator_pole = -10;

P.alpha = 0.2;

% state space design 
A = [...
    0, 1;...
    -P.k/P.m, -P.b/P.m;...
    ];
B = [0; 1/P.m ];
C = [...
    1, 0;...
    ];

% form augmented system
A1 = [A, zeros(2,1); -C, 0];
B1 = [B; 0];


% gain calculation
wn = 2.2/tr;
ol_char_poly=charpoly(A1);
des_char_poly = conv([1,2*zeta*wn,wn^2],...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(A1,B1))~=3, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)


