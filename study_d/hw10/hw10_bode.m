clc;clear;
massParamHW06
P.sigma = 0.1000;
P.ki = 0.5000;
P.kp = 3.8720;
P.kd = 5.7216;

options = bodeoptions;
options.MagUnits = 'abs';
options.MagScale = 'log';
G_num = [0.2];
G_den = [1,0.1,0.6];
G = tf(G_num,G_den);
figure(3), clf, 
[mag, phase, omegas] = bode(G,options);
bode(G,options);
grid on
omegas = sort([omegas;0.1;0.001;100;1000]);


C_num = [P.kd+P.sigma*P.kp,P.kp+P.sigma*P.ki,P.ki];
C_den = [P.sigma,1,0];
C = tf(C_num, C_den);
figure(4), clf, bode(G,omegas,options), grid on
hold on
bode(series(G,C),omegas,options)
legend('plant', 'PID control')
