classdef dataPlotterObserver_hw9 < handle
    properties
        % data histories
        time_history
        theta_history
        theta_hat_history
        theta_dot_history
        theta_hat_dot_history
        z_history
        z_hat_history
        z_dot_history
        z_hat_dot_history
        h_history
        h_hat_history
        h_dot_history
        h_hat_dot_history
        index
        % figure handles
        theta_handle
        theta_hat_handle
        theta_dot_handle
        theta_hat_dot_handle
        h_handle
        h_hat_handle
        h_dot_handle
        h_hat_dot_handle
        z_handle
        z_hat_handle
        z_dot_handle
        z_hat_dot_handle
        d_handle
        d_hat_handle
    end
    methods
        %--constructor--------------------------
        function self = dataPlotterObserver_hw9(P)
            % Instantiate lists to hold the time and data histories
            self.time_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.theta_hat_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.z_hat_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.h_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.h_hat_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.h_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.h_hat_dot_history = NaN*ones(1,(P.t_end-P.t_start)/P.t_plot);
            self.index = 1;

            % Create figure and axes handles
            figure(3), clf
            subplot(3,2,1)
                hold on
                self.z_handle = plot(self.time_history, self.z_history, 'g');
                self.z_hat_handle = plot(self.time_history, self.z_hat_history, 'b');
                ylabel('z(m)')
                title('Ballbeam State and Observed State')
            subplot(3,2,2)
                hold on
                self.z_dot_handle = plot(self.time_history, self.z_dot_history, 'g');
                self.z_hat_dot_handle = plot(self.time_history, self.z_hat_dot_history, 'b');
                ylabel('zdot(m/s)')
            subplot(3, 2, 3)
                hold on
                self.theta_handle = plot(self.time_history, self.theta_history, 'g');
                self.theta_hat_handle = plot(self.time_history, self.theta_hat_history, 'b');
                ylabel('theta(deg)')
            subplot(3, 2, 4)
                hold on
                self.theta_dot_handle = plot(self.time_history, self.theta_dot_history, 'g');
                self.theta_hat_dot_handle = plot(self.time_history, self.theta_hat_dot_history, 'b');
                ylabel('thetadot(deg/s)')
            subplot(3, 2, 5)
                hold on
                self.h_handle = plot(self.time_history, self.h_history, 'g');
                self.h_hat_handle = plot(self.time_history, self.h_hat_history, 'b');
                ylabel('h(m)')
            subplot(3, 2, 6)
                hold on
                self.h_dot_handle = plot(self.time_history, self.h_dot_history, 'g');
                self.h_hat_dot_handle = plot(self.time_history, self.h_hat_dot_history, 'b');
                ylabel('hdot(m/s)')
        end
        
        function self = update(self, t, x, xhat)
            % update the time history of all plot variables
            self.time_history(self.index) = t;
            self.z_history(self.index) = x(1);
            self.z_dot_history(self.index) = x(4);
            self.z_hat_history(self.index) = xhat(1);
            self.z_hat_dot_history(self.index) = xhat(4);
            self.theta_history(self.index) = 180/pi*x(3);
            self.theta_dot_history(self.index) = 180/pi*x(6);
            self.theta_hat_history(self.index) = 180/pi*xhat(3);
            self.theta_hat_dot_history(self.index) = 180/pi*xhat(6);
            self.h_history(self.index) = x(2);
            self.h_dot_history(self.index) = x(5);
            self.h_hat_history(self.index) = xhat(2);
            self.h_hat_dot_history(self.index) = xhat(5);
            self.index = self.index + 1;
            % update the plots with associated histories
            set(self.theta_handle, 'Xdata', self.time_history, 'Ydata', self.theta_history)
            set(self.theta_dot_handle, 'Xdata', self.time_history, 'Ydata', self.theta_dot_history)
            set(self.theta_hat_handle, 'Xdata', self.time_history, 'Ydata', self.theta_hat_history)
            set(self.theta_hat_dot_handle, 'Xdata', self.time_history, 'Ydata', self.theta_hat_dot_history)
            set(self.z_handle, 'Xdata', self.time_history, 'Ydata', self.z_history)
            set(self.z_dot_handle, 'Xdata', self.time_history, 'Ydata', self.z_dot_history)
            set(self.z_hat_handle, 'Xdata', self.time_history, 'Ydata', self.z_hat_history)
            set(self.z_hat_dot_handle, 'Xdata', self.time_history, 'Ydata', self.z_hat_dot_history)
            set(self.h_handle, 'Xdata', self.time_history, 'Ydata', self.h_history)
            set(self.h_dot_handle, 'Xdata', self.time_history, 'Ydata', self.h_dot_history)
            set(self.h_hat_handle, 'Xdata', self.time_history, 'Ydata', self.h_hat_history)
            set(self.h_hat_dot_handle, 'Xdata', self.time_history, 'Ydata', self.h_hat_dot_history)
        end
    end
end
