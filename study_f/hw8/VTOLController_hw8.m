classdef VTOLController_hw8 < handle
    %----------------------------
    properties
        Fe
        z_dot
        h_dot
        theta_dot
        z_d1
        h_d1
        theta_d1
        K_lon
        ki_lon
        K_lat
        ki_lat
        limit
        beta
        Ts
        integrator_z
        error_d1_z
        integrator_h
        error_d1_h
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = VTOLController_hw8(P)
            self.Fe = P.Fe;
            % initialized object properties
            self.z_dot = 0.0;
            self.h_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = 0.0;
            self.h_d1 = 0.0;
            self.theta_d1 = 0.0;
            self.K_lon = P.K_lon;
            self.ki_lon = P.ki_lon;
            self.K_lat = P.K_lat;
            self.ki_lat = P.ki_lat;
            self.limit = P.fmax;
            self.beta = P.beta;
            self.Ts = P.Ts;
            self.integrator_z = 0;
            self.error_d1_z = 0;
            self.integrator_h = 0;
            self.error_d1_h = 0;
        end
        %----------------------------
        function out = update(self, r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);

            % differentiate z and theta
            self.differentiateZ(z);
            self.differentiateH(h);
            self.differentiateTheta(theta);
            error_z = z_r - z;
            error_h = h_r - h;
            self.integrateErrorZ(error_z);
            self.integrateErrorH(error_h);

            % NOTE:  remember the feedback control should actually be
            % u_tilde = -K*x_tilde + kr*zd_tilde
            % since the only value that deviates from zero is z, we have
            x_lon = [h; self.h_dot];
            % equilibrium force
            Fe_ = self.Fe/cos(theta); 
                % divide Fe by cos(theta) so that force is right during
                % lateral translations.
            % compute the state feedback controller
            F_tilde = -self.K_lon*x_lon - self.ki_lon*self.integrator_h;
            F = Fe_ + F_tilde;

            % lateral control for position
            % construct the state
            x_lat = [z; theta; self.z_dot; self.theta_dot];
            % compute the state feedback controller
            tau = -self.K_lat*x_lat - self.ki_lat*self.integrator_z;
            
            out = [F; tau];
        end
        function self = integrateErrorZ(self, error)
            self.integrator_z = self.integrator_z...
                + (self.Ts/2.0)*(error+self.error_d1_z);
            self.error_d1_z = error;
        end
        function self = integrateErrorH(self, error)
            self.integrator_h = self.integrator_h...
                + (self.Ts/2.0)*(error+self.error_d1_h);
            self.error_d1_h = error;
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z - self.z_d1) / self.Ts);
            self.z_d1 = z;            
        end
        %----------------------------
        function self = differentiateH(self, h)
            self.h_dot = ...
                self.beta*self.h_dot...
                + (1-self.beta)*((h - self.h_d1) / self.Ts);
            self.h_d1 = h;            
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.theta_dot = ...
                self.beta*self.theta_dot...
                + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
            self.theta_d1 = theta;
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