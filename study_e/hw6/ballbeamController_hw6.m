classdef ballbeamController_hw6 < handle
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
        e_prev_in
        edot_prev_in
        e_prev_out
        edot_prev_out
        kiz
        kitheta
        z_prev
        zdot_prev
        theta_prev
        thetadot_prev
        ein_prev_in
        ein_prev
    end
    methods
        function self = ballbeamController_hw6(P)
            self.kpz = P.kpz;
            self.kdz = P.kdz;
            self.kiz = P.kiz;
            self.kitheta = P.kitheta;
            self.kptheta = P.kptheta;
            self.kdtheta = P.kdtheta;
            self.F_max = P.Fmax;
            % get all the param
            self.m1 = P.m1;
            self.g = P.g;
            self.length = P.length;
            self.m2 = P.m2;
            self.e_prev_in = 0;
            self.edot_prev_in = 0;
            self.e_prev_out = 0;
            self.edot_prev_out = 0;
            self.z_prev = 0;
            self.zdot_prev = 0;
            self.ein_prev_in = 0;
            self.theta_prev = 0;
            self.thetadot_prev = 0;
            self.ein_prev = 0;
        end
        function F = update(self, z_r, state, dt)
            z = state(1);
            theta = state(2);
            e_in = (z_r - z);
            edot_in = self.derivative(dt, e_in, self.e_prev_in, self.edot_prev_in);
            zdot = self.derivative(dt, z, self.z_prev, self.zdot_prev);
            eint_in = self.integrate(dt, e_in, self.e_prev_in, self.ein_prev_in);
            self.z_prev = z;
            self.zdot_prev = zdot;
            self.e_prev_in = e_in;
            self.edot_prev_in = edot_in;
            self.ein_prev_in = eint_in;
                        
            % compute equlibrium force
            F_e = self.m1*self.g*z/self.length + self.m2*self.g/2;
            % outer loop (since DC gain is 1, no need to multiply DC over
            % entire thing
            theta_r = self.kpz * e_in - self.kdz * zdot + self.kiz * eint_in;
            
            e_out = (theta_r - theta);
            edot_out = self.derivative(dt, e_out, self.e_prev_out, self.edot_prev_out);
            thetadot = self.derivative(dt, theta, self.theta_prev, self.thetadot_prev);
            eint_out = self.integrate(dt, e_out, self.e_prev_out, self.ein_prev);
            self.theta_prev = theta;
            self.thetadot_prev = thetadot;
            self.e_prev_out = e_out;
            self.edot_prev_out = edot_out;
            self.ein_prev = eint_out;
            
            % inner loop
            F = self.kptheta* e_out - self.kdtheta * thetadot + self.kitheta * eint_out;
            % compute total torque
            F = F_e + F;
            % saturate the final output being sent to the dynamics.
            F = self.saturate(F, self.F_max);
            %%%%%%%%%%%%%%% NEET ANTI-WINTUP %%%%%%%%%%% 
            %%%%%%%%%%%%%%% to prevent overshoot and fail %%%%%%%%%%%%%%%%%%%%
        end
        function zdot = derivative(self, dt, z, z_prev, zdot_prev)
            sigma = 0.05;
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