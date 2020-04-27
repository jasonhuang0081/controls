classdef massController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp
        kd
        limit
        k
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController(P)
            self.kp = P.kp; %this is the proportional gain
            self.kd = P.kd; %this is the derivative gain
            self.limit = P.F_max;
            self.k = P.k;  %this is the stiffness of the physical spring
        end
        %----------------------------
        function force = update(self, z_r, state)
            z = state(1);
            zdot = state(2);
            
            % calc equilibrium force 
            f_eq = z_r*self.k;
            
            % compute the linearized torque using PID
            force_tilde = self.kp * (z_r - z) - self.kd * zdot + f_eq;
            % compute total torque
            force = self.saturate(force_tilde);
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