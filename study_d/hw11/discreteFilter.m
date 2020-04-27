classdef discreteFilter < handle
    properties
        prev_filt_output
        prev_filt_input
        Ts
        num_d
        den_d
    end
    methods
        function self = discreteFilter(num, den, Ts)
            sys = tf(num, den);
            self.Ts = Ts;
            sys_d = c2d(sys, Ts, 'tustin');
            self.den_d = sys_d.Denominator{1};
            self.num_d = sys_d.Numerator{1};
            self.prev_filt_output = zeros(1, length(self.den_d)-1);
            self.prev_filt_input = zeros(1,length(self.num_d));
        end
        function y = update(self, u)
            % update filter values
            self.prev_filt_input = [u, self.prev_filt_input(1:end-1)];
            y = self.num_d*self.prev_filt_input' - self.den_d(2:end)*self.prev_filt_output';
            self.prev_filt_output = [y, self.prev_filt_output(1:end-1)];
        end
    end
end