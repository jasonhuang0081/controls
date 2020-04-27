classdef massControllerHW11 < handle
    %----------------------------
    properties
        control
        prefilter
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %----------------------------
        function self = massControllerHW11(P)
            % initialized object properties
            self.prefilter = discreteFilter(P.num_F, P.den_F, P.Ts);
            self.control = discreteFilter(P.num_C, P.den_C, P.Ts); 
            self.limit = P.F_max;
            self.Ts = P.Ts;
        end
        %----------------------------
        function force = update(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            
            % use the digital prefilter to smooth the reference input
            z_r_filtered = self.prefilter.update(z_r);

            % integrate error
            error = z_r_filtered - z;

            % Compute the control C(s)
            force_tilde = self.control.update(error);

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