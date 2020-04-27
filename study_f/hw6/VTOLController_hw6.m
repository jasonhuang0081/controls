classdef VTOLController_hw6 < handle
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
        e_prev_h
        edot_prev_h
        e_prev_z
        edot_prev_z
        e_prev_theta
        edot_prev_theta
        kiz
        kitheta
        kih
        ein_prev_h
        ein_prev_theta
        ein_prev_z
    end
    methods
        function self = VTOLController_hw6(P)
            self.kpz = P.kpz;
            self.kdz = P.kdz;
            self.kiz = P.kiz;
            self.kptheta = P.kptheta;
            self.kdtheta = P.kdtheta;
            self.kitheta = P.kitheta;
            self.kph = P.kph;
            self.kdh = P.kdh;
            self.kih = P.kih;
            self.F_max = P.fmax;
            self.Fe = P.Fe;
            self.mixing = P.mixing;
            self.e_prev_h = 0;
            self.edot_prev_h = 0;
            self.e_prev_z = 0;
            self.edot_prev_z = 0;
            self.e_prev_theta = 0;
            self.edot_prev_theta = 0;
            self.ein_prev_h = 0;
            self.ein_prev_z = 0;
            self.ein_prev_theta = 0;
        end
        function [u, F, tau] = update(self, z_r, h_r, state, dt)
            z = state(1);
            h = state(2);
            theta = state(3);
            e_h = (h_r - h);
            edot_h = self.derivative(dt, e_h, self.e_prev_h, self.edot_prev_h);
            eint_h = self.integrate(dt, e_h, self.e_prev_h, self.ein_prev_h);
            self.e_prev_h = e_h;
            self.edot_prev_h = edot_h;
            self.ein_prev_h = eint_h;
            
            % compute equlibrium force
            F_e = self.Fe;
            tau_e = 0;
                %longitude control (up-down)
            F = self.kph * e_h + self.kdh *edot_h + self.kih * eint_h;
                % laterial control (side way)
            % outer loop (since DC gain is 1, no need to multiply DC over
            % entire thing
            e_z = (z_r - z);
            edot_z = self.derivative(dt, e_z, self.e_prev_z, self.edot_prev_z);
            eint_z = self.integrate(dt, e_z, self.e_prev_z, self.ein_prev_z);
            self.e_prev_z = e_z;
            self.edot_prev_z = edot_z;
            self.ein_prev_z = eint_z;
            
            theta_r = self.kpz *e_z + self.kdz * edot_z + self.kiz * eint_z;
            % inner loop
            e_theta = (theta_r - theta);
            edot_theta = self.derivative(dt, e_theta, self.e_prev_theta, self.edot_prev_theta);
            eint_theta = self.integrate(dt, e_theta, self.e_prev_theta, self.ein_prev_theta);
            tau = self.kptheta* e_theta + self.kdtheta * edot_theta + self.kitheta * eint_theta;
            self.e_prev_theta = e_theta;
            self.edot_prev_theta = edot_theta;
            self.ein_prev_theta = eint_theta;
            % compute total torque
            F = F_e + F;
            tau = tau_e + tau;
            u = self.mixing*[F; tau];
            % saturate the final output being sent to the dynamics.
            u(1) = self.saturate(u(1), self.F_max);
            u(2) = self.saturate(u(2), self.F_max);
        end
        function zdot = derivative(self, dt, z, z_prev, zdot_prev)
            sigma = 0.005;
            zdot = (sigma*2 - dt)/(2*sigma + dt)*zdot_prev + 2/(2*sigma + dt)*(z - z_prev);
        end
        function integrated = integrate(self, dt, z, z_prev, zin_prev)
            integrated = zin_prev + dt/2*(z + z_prev);
        end
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end