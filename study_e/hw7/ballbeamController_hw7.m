classdef ballbeamController_hw7 < handle
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        thetaCtrl
        m1
        m2
        g
        length
        P
        z_dl
        z_dot
        th_dl
        th_dot
    end
    %----------------------------
    methods
        %----------------------------
        function self = ballbeamController_hw7(P)
            self.P = P;
            
            self.m1 = P.m1;
            self.m2 = P.m2;
            self.g = P.g;
            self.length = P.length;
            
            self.z_dl = 0;
            self.z_dot = 0;
            self.th_dl = 0;
            self.th_dot = 0;
        end

        %----------------------------
        function F = update(self, z_r, y)
            z = y(1);
            theta = y(2);
            self.differentiateZ(z);
            self.differentiateTheta(theta);
            z_e = self.length/2;

            F_tilde = (z_r-z_e)*self.P.kr - self.P.K*[z - z_e;theta;y(3);y(4)];
            % equilibrium force
            Fe = self.m1*self.g*(z_e/self.length) + self.m2*self.g/2;
            % total force
            F = self.saturate(F_tilde + Fe,self.P.Fmax);
        end
        function self = differentiateZ(self, z)
            self.z_dot = self.P.beta*self.z_dot + (1-self.P.beta)*((z-self.z_dl)/self.P.Ts);
            self.z_dl = z;
        end
        function self = differentiateTheta(self, th)
            self.th_dot = self.P.beta*self.th_dot + (1-self.P.beta)*((th-self.th_dl)/self.P.Ts);
            self.th_dl = th;
        end
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end