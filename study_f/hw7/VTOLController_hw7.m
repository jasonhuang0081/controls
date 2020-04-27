classdef VTOLController_hw7 < handle
    %----------------------------
    properties
        zCtrl
        hCtrl
        thetaCtrl
        Fe
        P
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController_hw7(P)
            self.Fe = P.Fe;
            self.P = P;
        end
        %----------------------------
        function out = update(self,z_r,h_r, y)

            z = y(1);
            h = y(2);
            theta = y(3);
            zdot = y(4);
            hdot = y(5);
            thetadot = y(6);   

            F_tilde = h_r*self.P.kr_ver - self.P.K_ver*[h;hdot];
            tau = z_r*self.P.kr_lat - self.P.K_lat*[z;theta;zdot;thetadot];
            
            F = F_tilde + self.Fe;

            out = [F; tau];
            u = self.P.mixing*out;
            u(1) = self.saturate(u(1), self.P.fmax);
            u(2) = self.saturate(u(2), self.P.fmax);
            out = u;
%             out = inv(self.P.mixing)*u;
        end
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end