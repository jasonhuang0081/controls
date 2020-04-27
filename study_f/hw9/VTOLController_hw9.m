classdef VTOLController_hw9 < handle
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
        tau_d1
        F_dl
        x_hat
        P
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = VTOLController_hw9(P)
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
            self.P = P;
            
            self.tau_d1 = 0;
            self.x_hat = [0;0;0;0;0;0];
            self.F_dl = 0;
        end
        %----------------------------
        function [out,xyhat] = update(self, r, y)
            xyhat = self.updateObserver(y);         
            
            z_r = r(1);
            h_r = r(2);
            z = xyhat(1);
            h = xyhat(2);
            theta = xyhat(3);

%             self.differentiateZ(z);
%             self.differentiateH(h);
%             self.differentiateTheta(theta);
            error_z = z_r - z;
            error_h = h_r - h;
            self.integrateErrorZ(error_z);
            self.integrateErrorH(error_h);


            x_lon = [h;xyhat(5)];
            % equilibrium force
            Fe_ = self.Fe/cos(theta); 
%             Fe_ = self.Fe; 
%             Fe_ = 0;

            % compute the state feedback controller
            F_tilde = -self.K_lon*x_lon - self.ki_lon*self.integrator_h;
            F = Fe_ + self.saturate(F_tilde,self.P.Ftildemax);

            % lateral control for position
            % construct the state
            x_lat = [z; theta; xyhat(4); xyhat(6)];
            % compute the state feedback controller
            unsat_tau = (-self.K_lat*x_lat - self.ki_lat*self.integrator_z);
            tau = self.saturate(unsat_tau,self.P.taumax);
            
            out = [F; tau];
            self.integratorAntiWindup_lon(F,F_tilde + Fe_);
            self.integratorAntiWindup_lat(tau,unsat_tau);
            self.tau_d1 = unsat_tau;
            self.F_dl = F_tilde;
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
        function xhat = updateObserver(self, y)
            F1 = self.observer_f(self.x_hat, y);
            F2 = self.observer_f(self.x_hat + self.Ts/2*F1, y);
            F3 = self.observer_f(self.x_hat + self.Ts/2*F2, y);
            F4 = self.observer_f(self.x_hat + self.Ts*F3, y);
            self.x_hat = self.x_hat + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            xhat = self.x_hat;
        end
        function x_hat_dot = observer_f(self, x_hat_all, y_all)
            xhat = x_hat_all([1,3,4,6]);
            y = y_all([1,3]);
            x_e = [0; 0;0;0];
            y_e = [0;0];
            F_e = 0;
            x_hat_dot1 = self.P.A_lat1 * (xhat-x_e)...
                + self.P.B_lat1*(self.tau_d1 - F_e)...
                + self.P.L_lat*((y-y_e) - self.P.C_lat * (xhat-x_e));
            
            y2 = y_all([2]);
            x_hat2 = x_hat_all([2,5]);
            x_e2 = [0; 0];
            y_e2 = [0];
%             F_e2 = self.Fe; 

            x_hat_dot2 = self.P.A_lon1 * (x_hat2-x_e2)...
                + self.P.B_lon1*(self.F_dl)...
                + self.P.L_lon*((y2-y_e2) - self.P.C_lon * (x_hat2-x_e2));
            
            x_hat_dot = [x_hat_dot1(1);x_hat_dot2(1);x_hat_dot1(2);
                x_hat_dot1(3);x_hat_dot2(2);x_hat_dot1(4)];
        end
        function self = integratorAntiWindup_lon(self, u_sat, u_unsat)
            % integrator anti-windup
            if self.ki_lon~=0
                self.integrator_h = self.integrator_h + self.Ts/self.ki_lon*(u_sat-u_unsat);
            end
        end
        function self = integratorAntiWindup_lat(self, u_sat, u_unsat)
            % integrator anti-windup
            if self.ki_lat~=0
                self.integrator_z = self.integrator_z + self.Ts/self.ki_lat*(u_sat-u_unsat);
            end
        end
        %----------------------------
%         function self = differentiateZ(self, z)
%             self.z_dot = ...
%                 self.beta*self.z_dot...
%                 + (1-self.beta)*((z - self.z_d1) / self.Ts);
%             self.z_d1 = z;            
%         end
%         %----------------------------
%         function self = differentiateH(self, h)
%             self.h_dot = ...
%                 self.beta*self.h_dot...
%                 + (1-self.beta)*((h - self.h_d1) / self.Ts);
%             self.h_d1 = h;            
%         end
%         %----------------------------
%         function self = differentiateTheta(self, theta)
%             self.theta_dot = ...
%                 self.beta*self.theta_dot...
%                 + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
%             self.theta_d1 = theta;
%         end
        %----------------------------
        function out = saturate(self,u,limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end