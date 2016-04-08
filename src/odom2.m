% Odometry model used in EJ Kreiner's thesis.
function x_new = odom2(dt,x)
x_new = [x(1) + x(4)*dt.*cos(x(3)+x(5)*dt/2);
         x(2) + x(4)*dt.*sin(x(3)+x(5)*dt/2);
         x(3) + x(5)*dt;
         x(4) + 0*dt; %0*dt used for dt of variable length
         x(5) + 0*dt];
end