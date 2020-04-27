classdef massController_hw
    properties
        kp
        kd
        F_max
        ze
        k
    end
    methods
        function self = massController_hw(P)
            self.kp = P.kp;
            self.kd = P.kd;
            self.F_max = P.F_max;
            % plant parameters known to controller
            self.ze = P.z0;   % z is moving, so can't use this all the time
            self.k = P.k;
        end
        function F = update(self, z_r, state)
            z = state(1);
            zdot = state(2);
            % compute equlibrium force
            Fe = self.k*z_r;
            % compute the linearized torque using PID
            F = self.kp * (z_r - z) - self.kd * zdot;
            % compute total torque
            F = Fe + F;
            % saturate the final output being sent to the dynamics.
            F = self.saturate(F, self.F_max);
        end
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end