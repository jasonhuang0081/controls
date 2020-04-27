% ballbeam parameter file
ballbeamParam % general parameters

% load PID parameters from HW 06
ballbeamParamHW06

P_in = tf([b0],[1,0,0]);
P_out = tf(-P.g,[1,0,0]);

C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_z+P.kp_z*P.sigma),(P.kp_z+P.ki_z*P.sigma),P.ki_z],[P.sigma,1,0]);


figure(2), clf, 
bode(P_in), grid on
hold on
bode(series(C_in,P_in))
legend('No control - P(s)', 'C(s)*P(s)')
title('Ballbeam, Inner Loop')

figure(3), clf, 
bode(P_out), grid on
hold on
bode(series(C_out,P_out))
legend('No control', 'C(s)*P(s)')
title('Ballbeam, Outer Loop')

