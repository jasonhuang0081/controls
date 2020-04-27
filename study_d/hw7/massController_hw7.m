classdef massController_hw7 < handle
    properties
        zCtrl
        limit
        P
        y_d1
        k
        y_dot
    end
    methods
        %----------------------------
        function self = massController_hw7(P)
            self.P = P;
            self.k = P.k; 
            self.limit = P.F_max;
            self.y_d1 = 0.0;
            self.y_dot = 0;
            
        end
        %----------------------------
        function force = update(self, z_r, y)
            z = y(1);
            self.differentiateY(z);      
            f_eq = z_r*self.k;
            force_tilde = z_r*self.P.kr - self.P.K*[z;self.y_dot];
            force = self.saturate(force_tilde);
        end
        function self = differentiateY(self, y)
            self.y_dot = self.P.beta*self.y_dot + (1-self.P.beta)*((y-self.y_d1)/self.P.Ts);
            self.y_d1 = y;
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