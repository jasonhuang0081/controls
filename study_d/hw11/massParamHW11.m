massParam

% load parameters from HW06 D.10
massParamHW06

% transfer function for robot arm
Plant = tf([1/P.m],[1, P.b/P.m, P.k/P.m]);
C_pid = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0]);

% margin and bode plots 
figure(1), clf, margin(Plant*C_pid), grid on, hold on
bode(Plant*C_pid/(1+Plant*C_pid)) 
legend('Open Loop', 'Closed Loop')





