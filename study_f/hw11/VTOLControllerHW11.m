classdef VTOLControllerHW11 < handle
    %----------------------------
    properties
        Fe
        control_lat_in
        control_lat_out
        control_lon
        filter_lon
        filter_lat
        limit
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = VTOLControllerHW11(P)
            self.Fe = P.Fe;
            self.control_lat_in = discreteFilter(P.num_Clat_in,P.den_Clat_in, P.Ts);
            self.control_lat_out = discreteFilter(P.num_Clat_out, P.den_Clat_out, P.Ts);
            self.control_lon = discreteFilter(P.num_Clon,P.den_Clon, P.Ts);
            self.filter_lon = discreteFilter(P.num_Flon,P.den_Flon, P.Ts);
            self.filter_lat = discreteFilter(P.num_Flat,P.den_Flat, P.Ts);
            self.limit = P.fmax;
            self.Ts = P.Ts;
        end
        %----------------------------
        function out = u(self, r, y)
            % y_r is the referenced input
            % y is the current state
             z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);
            
            %-----Longitudinal Control------
            % solve differential equation defining prefilter
            h_r_filtered = self.filter_lon.update(h_r);
                
            % error signal for longitudinal loop
            error_lon = h_r_filtered - h;
                
            % implement longitudinal controller
            F_tilde = self.control_lon.update(error_lon);
            
            %-----Lateral Control------
            %+++++Lateral Outer Loop+++++
            z_r_filtered = self.filter_lat.update(z_r);
                
            % error signal for outer loop
            error_lat_out = z_r_filtered - z;
                
            % Outer loop control C_out
            theta_r = self.control_lat_out.update(error_lat_out);

            %+++++Lateral Inner Loop+++++
            % error signal for inner loop
            error_lat_in = theta_r - theta;
                
            % Inner loop control C_in
            tau = self.control_lat_in.update(error_lat_in);  

            % total force
            F = F_tilde + self.Fe;
            out = [F; tau];
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