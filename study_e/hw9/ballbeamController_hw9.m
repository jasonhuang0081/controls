classdef ballbeamController_hw9< handle
    %----------------------------
    properties
        m1
        m2
        g
        length
        init_flag
        z_dot
        theta_dot
        z_d1
        theta_d1
        K
        ki
        limit
        beta
        error_d1
        integrator
        Ts
        z_e
        tau_d1
        F_e
        x_hat
        P
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = ballbeamController_hw9(P)
            self.m1 = P.m1;
            self.m2 = P.m2;
            self.g = P.g;
            self.length = P.length;
            % initialized object properties
            self.error_d1 = 0;
            self.integrator = 0;
            self.init_flag = 1;
            self.z_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = 0.0;
            self.theta_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.Fmax;
            self.beta = P.beta;
            self.Ts = P.Ts;
            self.F_e = 0;
            self.tau_d1 = 0;
            self.P = P;
              
            self.z_e = self.length/2;
            self.x_hat = [0;0;0;0];
        end
        %----------------------------
        function [F,xhat] = update(self, z_r, y)
            self.F_e = 0.5*self.m2*self.g + self.m1*self.g*self.z_e/self.length;
            xhat = self.updateObserver(y);
            zhat = xhat(1);

            error = z_r - zhat;
            self.integrateError(error);
            xe = [self.length/2; 0; 0; 0];
            F_tilde = -self.K*(xhat-xe) - self.ki*self.integrator;
            
            % total force
            F_unsat = F_tilde + self.F_e;
            F = self.saturate(F_unsat);
            self.integratorAntiWindup(F,F_unsat);
            self.tau_d1 = F_unsat;
        end
        %----------------------------
        function xhat = updateObserver(self, y)
            F1 = self.observer_f(self.x_hat, y);
            F2 = self.observer_f(self.x_hat + self.Ts/2*F1, y);
            F3 = self.observer_f(self.x_hat + self.Ts/2*F2, y);
            F4 = self.observer_f(self.x_hat + self.Ts*F3, y);
            self.x_hat = self.x_hat + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            xhat = self.x_hat;
        end
        function x_hat_dot = observer_f(self, x_hat, y)
            % compute feedback linearizing torque tau_fl
            %             theta_hat = x_hat(1);
            x_e = [self.z_e; 0;0;0];
            y_e = [self.z_e;0];
            x_hat_dot = self.P.A1 * (x_hat-x_e)...
                + self.P.B1*(self.tau_d1 - self.F_e)...
                + self.P.L*((y-y_e) - self.P.C * (x_hat-x_e));
        end
        function self = integrateError(self, error)
            self.integrator = self.integrator...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function self = integratorAntiWindup(self, u_sat, u_unsat)
            % integrator anti-windup
            if self.ki~=0
                self.integrator = self.integrator + self.Ts/self.ki*(u_sat-u_unsat);
            end
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end