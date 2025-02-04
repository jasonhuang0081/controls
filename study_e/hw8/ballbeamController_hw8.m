classdef ballbeamController_hw8< handle
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
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = ballbeamController_hw8(P)
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
        end
        %----------------------------
        function F = update(self, z_r, y)
            z = y(1);
            theta = y(2);

            % differentiate z and theta
            if self.init_flag ==1
                self.z_d1 = z;
                self.theta_d1 = theta;
                self.init_flag = 0;
            end
            error = z_r - z;
            self.integrateError(error);
            self.differentiateZ(z);
            self.differentiateTheta(theta);

            z_e = self.length/2;
            % NOTE:  remember the feedback control should actually be
            % u_tilde = -K*x_tilde + kr*zd_tilde
            % since the only value that deviates from zero is z, we have
            x_tilde = [z-z_e; theta; self.z_dot; self.theta_dot];
            % equilibrium force
            F_e = 0.5*self.m2*self.g + self.m1*self.g*z_e/self.length;
            % compute the state feedback controller
%             zr_tilde = z_r - z_e;
            F_tilde = -self.K*x_tilde - self.ki*self.integrator;
            
            % total force
            F_unsat = F_tilde + F_e;
            F = self.saturate(F_unsat);
            self.integratorAntiWindup(F,F_unsat);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z - self.z_d1) / self.Ts);
            self.z_d1 = z;            
        end
        function self = integrateError(self, error)
            self.integrator = self.integrator...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.theta_dot = ...
                self.beta*self.theta_dot...
                + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
            self.theta_d1 = theta;
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