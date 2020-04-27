classdef VTOLDynamics_hw < handle
    %  Model the physical system
    %----------------------------
    properties
        state
        mr
        mc
        Jc
        d
        mu
        g
        Ts
    end
    %----------------------------
    methods
        %---constructor-------------------------
        function self = VTOLDynamics_hw(P)
            % Initial state conditions
            self.state = [...
                        P.z0;...                             
                        P.h0;...
                        P.theta0;...
                        P.zdot0;...
                        P.hdot0;...
                        P.thetadot0...
                        ]; 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The parameters for any physical system are never known exactly.  Feedback
            % systems need to be designed to be robust to this uncertainty.  In the simulation
            % we model uncertainty by changing the physical parameters by a uniform random variable
            % that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
            % may change by up to 20%.  A different parameter value is chosen every time the simulation
            % is run.
            alpha = P.alpha;  % Uncertainty parameter
            self.mr = P.mr * (1+2*alpha*rand-alpha);  % Mass of the arm, kg
            self.mc = P.mc * (1+2*alpha*rand-alpha);  % Length of the arm, m
            self.Jc = P.Jc * (1+2*alpha*rand-alpha);  % Damping coefficient, Ns
            self.d = P.d * (1+2*alpha*rand-alpha); 
            self.mu = P.mu * (1+2*alpha*rand-alpha); 
            self.g = P.g;  % the gravity constant is well known and so we don't change it.
            self.Ts = P.Ts; % sample rate at which dynamics is propagated
        end
        %----------------------------
        function y = update(self, u)
            self.rk4_step(u);
            y = self.h();
        end
        %----------------------------
        function self = rk1_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK1 algorithm
            self.state = self.state + self.Ts * self.f(self.state, u);
        end
        %----------------------------
        function self = rk2_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK2 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2 * F1, u);
            self.state = self.state + self.Ts/6 * (F1 + F2);
        end
        %----------------------------
        function self = rk4_step(self, u)
            %
            % Integrate the differential equations defining dynamics
            % P.Ts is the time step between function calls.
            % u contains the system input(s).
            % 
            % Integrate ODE using Runge-Kutta RK4 algorithm
            F1 = self.f(self.state, u);
            F2 = self.f(self.state + self.Ts/2*F1, u);
            F3 = self.f(self.state + self.Ts/2*F2, u);
            F4 = self.f(self.state + self.Ts*F3, u);
            self.state = self.state + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
        end
        
        %----------------------------
        function xdot = f(self, state, u)
            %
            % Return xdot = f(x,u), the derivatives of the continuous states, as a matrix
            % 
            % re-label states and inputs for readability
            z = state(1);
            h = state(2);
            theta = state(3);
            zdot = state(4);
            hdot = state(5);
            thetadot = state(6);
            fr = u(1);
            fl = u(2);
            % The equations of motion.
            zddot = (-fr*sin(theta)-fl*sin(theta)-self.mu*zdot) * (1/(self.mc + 2*self.mr));
            hddot = (fr*cos(theta)+fl*cos(theta)-2*self.mr*self.g-self.mc*self.g) * (1/(self.mc+2*self.mr));
            thetaddot = ((fr - fl)*self.d) * (1/(self.Jc + 2*self.mc*self.d^2));

            % build xdot and return
            xdot = [zdot; hdot; thetadot; zddot; hddot; thetaddot];
        end
        %----------------------------
        function y = h(self)
            %
            % Returns the measured outputs as a list
            % [z, theta] with added Gaussian noise
            % 
            % re-label states for readability
            z = self.state(1);
            h = self.state(2);
            theta = self.state(3);
            % add Gaussian noise to outputs
            %theta_m = theta + 0.001*randn;
            % return measured outputs
            y = [z; h; theta];
        end
    end
end


