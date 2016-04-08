% Odometry model described in Wang 1988.
% Doesn't currently accept dt as a column vector. This could be fixed!
function x_new = odom3(dt,x)
% Determine the correction factor detailed in Wang 1988.
cf = sin(x(5)*dt/2)./(x(5)*dt/2);
cf(isnan(cf)) = 1;
cf = [cf; cf; zeros(size(cf)); zeros(size(cf)); zeros(size(cf))];
s = size(x);
if (s(1)<s(2))
    x = x';
end
x_new = x + cf.*[x(4).*dt.*cos(x(3)+x(5)*dt/2);
                 x(4).*dt.*sin(x(3)+x(5)*dt/2);
                 x(5)*dt;
                 0*dt; %0*dt used for dt of variable length
                 0*dt];
end