classdef massController_hw8 < handle
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
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController_hw8(P)
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
        end
        %----------------------------
        function force = update(self, z_r, y)
            z = y(1);
            self.differentiateZ(z);
            %             self.z_dot = y(2);
            error = z_r - z;
            self.integrateError(error);
            x = [z; self.z_dot];
            
            % Compute the state feedback controller
            force_tilde = -self.K*x - self.ki*self.integrator;
            
            % compute total torque
            force = self.saturate(force_tilde);
            self.integratorAntiWindup(force,force_tilde);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z-self.z_d1) / self.Ts);
            self.z_d1 = z;
        end
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