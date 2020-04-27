classdef massController_hw6 < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp
        kd
        ki
        limit
        k
        e_prev
        edot_prev
        z_prev
        zdot_prev
        ein_prev
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController_hw6(P)
            self.kp = P.kp; %this is the proportional gain
            self.kd = P.kd; %this is the derivative gain
            self.ki = P.ki;
            self.limit = P.F_max;
            self.k = P.k;  %this is the stiffness of the physical spring
            self.e_prev = 0;
            self.edot_prev = 0;
            self.z_prev = 0;
            self.zdot_prev = 0;
            self.ein_prev = 0;
        end

        %----------------------------
        function force = update(self, z_r, state, dt)
            z = state(1);
            error = (z_r - z);
            edot = self.derivative(dt, error, self.e_prev, self.edot_prev);
            zdot = self.derivative(dt, z, self.z_prev, self.zdot_prev);
            eint = self.integrate(dt, error, self.e_prev, self.ein_prev);
            self.z_prev = z;
            self.zdot_prev = zdot;
            self.e_prev = error;
            self.edot_prev = edot;
            self.ein_prev = eint;
            % calc equilibrium force 
            f_eq = z_r*self.k;
            
            % compute the linearized torque using PID
            force_tilde = self.kp*error + self.kd*edot + self.ki*eint + f_eq;
            % compute total torque
            force = self.saturate(force_tilde);
            %%%%%%%%%%%%%%% NEET ANTI-WINTUP %%%%%%%%%%% 
            %%%%%%%%%%%%%%% to prevent overshoot and fail %%%%%%%%%%%%%%%%%%%%
        end
        %----------------------------
        function zdot = derivative(self, dt, z, z_prev, zdot_prev)
            sigma = 0.05;
            zdot = (sigma*2 - dt)/(2*sigma + dt)*zdot_prev + 2/(2*sigma + dt)*(z - z_prev);
        end
        function integrated = integrate(self, dt, z, z_prev, zin_prev)
            integrated = zin_prev + dt/2*(z + z_prev);
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end