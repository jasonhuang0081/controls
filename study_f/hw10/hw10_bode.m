clc;clear;
VTOLParamHW06
options = bodeoptions;
options.MagUnits = 'abs';
options.MagScale = 'log';
%% F to H
G = tf(1,[P.mc+2*P.mr,0,0]);
figure(3), clf, 
bode(G,options);
grid on

[mag, phase, omegas] = bode(G);
omegas = sort([omegas;0.1;30;100]);


C_num = [P.kd_h+P.sigma*P.kp_h,P.kp_h+P.sigma*P.ki_h,P.ki_h];
C_den = [P.sigma,1,0];
C = tf(C_num, C_den);
figure(4), clf, bode(G,omegas,options), grid on
hold on
bode(series(G,C),omegas,options)
legend('plant', 'PID control')
grid on
%% tau to theta
G = tf(1,[P.Jc+2*P.mr*P.d^2,0,0]);
figure(3), clf, 
bode(G,options);
grid on

[mag, phase, omegas] = bode(G);
omegas = sort([omegas;2;30;100]);

P.ki_th = 0;   % no ki_th
C_num = [P.kd_th+P.sigma*P.kp_th,P.kp_th+P.sigma*P.ki_th,P.ki_th];
C_den = [P.sigma,1,0];
C = tf(C_num, C_den);
figure(4), clf, bode(G,omegas,options), grid on
hold on
bode(series(G,C),omegas,options)
legend('plant', 'PID control')
grid on
%% theta to z
Fe = (P.mc+2*P.mr)*P.g;
G = tf(-Fe,[P.mc+2*P.mr,P.mu,0]);
figure(3), clf, 
bode(G,options);
grid on

[mag, phase, omegas] = bode(G);
omegas = sort([omegas;2;30;100]);

C_num = [P.kd_z+P.sigma*P.kp_z,P.kp_z+P.sigma*P.ki_z,P.ki_z];
C_den = [P.sigma,1,0];
C = tf(C_num, C_den);
figure(4), clf, bode(G,omegas,options), grid on
hold on
bode(series(G,C),omegas,options)
legend('plant', 'PID control')
grid on


