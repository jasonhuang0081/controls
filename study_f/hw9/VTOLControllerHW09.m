classdef VTOLControllerHW09 < handle
    %----------------------------
    properties
        Fe
        xhat_lon
        xhat_lat
        K_lon
        ki_lon
        K_lat
        ki_lat
        L_lon
        A_lon
        B_lon
        C_lon
        L_lat
        A_lat
        B_lat
        C_lat       
        integrator_z
        error_z_d1
        integrator_h
        error_h_d1
        F_d1
        tau_d1
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = VTOLControllerHW09(P)
            self.Fe = P.Fe;
            self.xhat_lon = [0; 0];
            self.xhat_lat = [0; 0; 0; 0];
            self.K_lon = P.K_lon;
            self.ki_lon = P.ki_lon;
            self.K_lat = P.K_lat;
            self.ki_lat = P.ki_lat;
            self.L_lon = P.L_lon;    
            self.A_lon = P.A_lon;    
            self.B_lon = P.B_lon;    
            self.C_lon = P.C_lon;    
            self.L_lat = P.L_lat;    
            self.A_lat = P.A_lat;    
            self.B_lat = P.B_lat;    
            self.C_lat = P.C_lat;    
            self.integrator_z = 0;
            self.error_z_d1 = 0;
            self.integrator_h = 0;
            self.error_h_d1 = 0;
            self.F_d1 = 0;
            self.tau_d1 = 0;
            self.limit = P.fmax;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function out  = update(self, r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);

            % update the observers 
            self.updateObserver(y);
            h_hat = self.xhat_lon(1);
            z_hat = self.xhat_lat(1);
            
            % integrate error
            error_z = z_r - z_hat;
            self.integrateErrorZ(error_z);
            error_h = h_r - h_hat;
            self.integrateErrorH(error_h);
            

            % NOTE:  remember the feedback control should actually be
            % u_tilde = -K*x_tilde + kr*zd_tilde
            % since the only value that deviates from zero is z, we have
            % equilibrium force
            Fe_ = self.Fe/cos(theta); 
                % divide Fe by cos(theta) so that force is right during
                % lateral translations.
            % compute the state feedback controller
            F = Fe_ - self.K_lon*self.xhat_lon - self.ki_lon*self.integrator_h;

            % lateral control for position
            % compute the state feedback controller
            tau = -self.K_lat*self.xhat_lat - self.ki_lat*self.integrator_z;
            self.updateForce(F);
            self.updateTorque(tau);
            
            out = [F; tau];
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            z_m = y_m(1);
            h_m = y_m(2);
            theta_m = y_m(3);
            N = 10;
            for i=1:N
                % observer for longitudinal states
                self.xhat_lon = self.xhat_lon + ...
                    self.Ts/N*(self.A_lon*self.xhat_lon+self.B_lon*(self.F_d1-self.Fe)...
                            + self.L_lon*(h_m-self.C_lon*self.xhat_lon));
                % observer for lateral states
                self.xhat_lat = self.xhat_lat + ...
                    self.Ts/N*(self.A_lat*self.xhat_lat+self.B_lat*self.tau_d1...
                        +self.L_lat*([z_m;theta_m]-self.C_lat*self.xhat_lat));
            end
        end
        %----------------------------
        function self = updateForce(self, F)
            self.F_d1 = F;
        end
        %----------------------------
        function self = updateTorque(self, tau)
            self.tau_d1 = tau;
        end
        %----------------------------
        function self = integrateErrorZ(self, error_z)
            self.integrator_z = self.integrator_z + (self.Ts/2.0)*(error_z+self.error_z_d1);
            self.error_z_d1 = error_z;
        end
        %----------------------------
        function self = integrateErrorH(self, error_h)
            self.integrator_h = self.integrator_h + (self.Ts/2.0)*(error_h+self.error_h_d1);
            self.error_h_d1 = error_h;
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