classdef VTOLController_hw
    properties
        kpz
        kdz
        kptheta
        kdtheta
        F_max
        Fe
        kph
        kdh
        mixing
    end
    methods
        function self = VTOLController_hw(P)
            self.kpz = P.kpz;
            self.kdz = P.kdz;
            self.kptheta = P.kptheta;
            self.kdtheta = P.kdtheta;
            self.kph = P.kph;
            self.kdh = P.kdh;
            self.F_max = P.fmax;
            self.Fe = P.Fe;
            self.mixing = P.mixing;
        end
        function [u, F, tau] = update(self, z_r, h_r, state)
            z = state(1);
            h = state(2);
            theta = state(3);
            zdot = state(4);
            hdot = state(5);
            thetadot = state(6);
            % compute equlibrium force
            F_e = self.Fe;
            tau_e = 0;
                %longitude control (up-down)
            F = self.kph * (h_r - h) - self.kdh *hdot;
                % laterial control (side way)
            % outer loop (since DC gain is 1, no need to multiply DC over
            % entire thing
            theta_r = self.kpz * (z_r - z) - self.kdz * zdot;
            % inner loop
            tau = self.kptheta* (theta_r - theta) - self.kdtheta * thetadot;
            % compute total torque
            F = F_e + F;
            tau = tau_e + tau;
            u = self.mixing*[F; tau];
            % saturate the final output being sent to the dynamics.
            u(1) = self.saturate(u(1), self.F_max);
            u(2) = self.saturate(u(2), self.F_max);
        end
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end