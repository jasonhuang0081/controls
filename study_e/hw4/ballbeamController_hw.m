classdef ballbeamController_hw
    properties
        kpz
        kdz
        kptheta
        kdtheta
        F_max
        m1
        g
        length
        m2
    end
    methods
        function self = ballbeamController_hw(P)
            self.kpz = P.kpz;
            self.kdz = P.kdz;
            self.kptheta = P.kptheta;
            self.kdtheta = P.kdtheta;
            self.F_max = P.Fmax;
            % get all the param
            self.m1 = P.m1;
            self.g = P.g;
            self.length = P.length;
            self.m2 = P.m2;
        end
        function F = update(self, z_r, state)
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            % compute equlibrium force
            F_e = self.m1*self.g*z/self.length + self.m2*self.g/2;
            % outer loop (since DC gain is 1, no need to multiply DC over
            % entire thing
            theta_r = self.kpz * (z_r - z) - self.kdz * zdot;
            % inner loop
            F = self.kptheta* (theta_r - theta) - self.kdtheta * thetadot;
            % compute total torque
            F = F_e + F;
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