% mass - parameter file
massParam % general parameters

% uncertainty parameter
P.alpha = 0.0;

% tuning parameters
tr = 2.5;  % tuned value of the rise time
zeta = 0.707;
wn = 2.2/tr;

A = [0 1;
    -P.k/P.m -P.b/P.m];
B = [0; 1/P.m];
C = [1 0];

des_char_poly = [1,2*zeta*wn,wn^2];
des_poles = roots(des_char_poly)

if rank(ctrb(A,B))~=2 
    disp('System Not Controllable'); 
else
    P.K = place(A,B,des_poles); 
    P.kr = -1/(C*inv(A-B*P.K)*B);
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t kr: %f\n', P.kr)


