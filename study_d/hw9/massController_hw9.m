classdef massController_hw9 < handle
    %----------------------------
    properties
        z_dot
        z_d1
        K
        ki
        limit
        beta
        Ts
        error_d1
        integrator
        x_hat
        P
        tau_d1
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController_hw9(P)
            % initialized object properties
            self.z_dot = 0.0;
            self.z_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.F_max;
            self.integrator = 0;
            self.error_d1 = 0;
            self.beta = P.beta;
            self.Ts = P.Ts;
            self.x_hat = [0.0; 0.0];
            self.P = P;
            self.tau_d1 = 0;
        end
        %----------------------------
        function [force,xhat] = update(self, z_r, y)
            xhat = self.updateObserver(y);
            z_hat = xhat(1);            
                     
%             self.differentiateZ(z_hat);
            %             self.z_dot = y(2);
            error = z_r - z_hat;
            self.integrateError(error);
            
            % Compute the state feedback controller
            force_tilde = -self.K*xhat - self.ki*self.integrator;
            
            % compute total torque
            force = self.saturate(force_tilde);
            self.integratorAntiWindup(force,force_tilde);
            self.tau_d1 = force; 
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
            x_e = [0; 0];
            y_e = 0;
            tau_fl = 0;
            x_hat_dot = self.P.A1 * (x_hat-x_e)...
                + self.P.B1*(self.tau_d1 - tau_fl)...
                + self.P.L*((y-y_e) - self.P.C * (x_hat-x_e));
        end
%         function self = differentiateZ(self, z)
%             self.z_dot = ...
%                 self.beta*self.z_dot...
%                 + (1-self.beta)*((z-self.z_d1) / self.Ts);
%             self.z_d1 = z;
%         end
        function self = integrateError(self, error)
            self.integrator = self.integrator...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
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