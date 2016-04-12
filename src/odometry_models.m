%% odometry_models.m
% x (a vector) is the state of the robot. The elements of the vector, in
% order are x position (x), y position (y), heading (theta), linear
% velocity (v), angular velocity (omega)

%% Plot an example to illustrate how each algorithm diverges at a different time scale.

% Set initial conditions
xp(1) = 0;
xp(2) = 0;
xp(3) = pi/2+pi/8;
xp(4) = 2.5;
xp(5) = -1.2;
% Let time go from 0 to 3 seconds
t = linspace(0,3,301);

% Calculate the state estimate for each algorithm.
x1 = odom1(t,xp);
x2 = odom2(t,xp);
x3 = odom3(t,xp);

% Set some graphing parameters...
title_size = 20;
label_size = 16;
tick_size = 14;

% ... including plot boundries. These are also used to plot inner boxes to
% show the boundaries of zoomed in plots on the more zoomed out  plots.
x1_max = 8;
x1_min = -3.5;
y1_max = 10;
y1_min = -1.5;
% Make sure the boundaries are square
if (x1_max-x1_min ~= y1_max-y1_min)
    error('Graph 1 boundaries are not equal');
end
x2_max = 1.25;
x2_min = -1.6;
y2_max = 2.6;
y2_min = -0.25;
% Make sure the boundaries are square
if (x2_max-x2_min ~= y2_max-y2_min)
    error('Graph 2 boundaries are not equal');
end
x3_max = 0.09;
x3_min = -0.18;
y3_max = 0.25;
y3_min = -0.02;
% Make sure the boundaries are square
if (x3_max-x3_min ~= y3_max-y3_min)
    error('Graph 3 boundaries are not equal');
end

figure(1)
subplot(1,3,1)
N=301;
plot(x1(1,1:N),x1(2,1:N),x2(1,1:N),x2(2,1:N),x3(1,1:N),x3(2,1:N),'LineWidth',2)
hold on
% Plot the boundaries of plot 2 on plot 1.
plot([x2_min x2_max x2_max x2_min x2_min],[y2_min y2_min y2_max y2_max y2_min],'-k','LineWidth',1)
hold off
xlabel('x (m)','FontSize',label_size)
ylabel('y (m)','FontSize',label_size)
title1 = sprintf('%.2f seconds',t(N));
title(title1,'FontSize',title_size)
set(gca,'LineWidth',1.2,'FontSize',tick_size)
leg1 = 'A';
leg2 = 'B';
leg3 = 'C';
legend(leg1,leg2,leg3,'Location','northeast')
axis equal
xlim([x1_min x1_max])
ylim([y1_min y1_max])


subplot(1,3,2)
N = 101;
plot(x1(1,1:N),x1(2,1:N),x2(1,1:N),x2(2,1:N),x3(1,1:N),x3(2,1:N),'LineWidth',2)
hold on
% Plot the boundaries of plot 3 on plot 2.
plot([x3_min x3_max x3_max x3_min x3_min],[y3_min y3_min y3_max y3_max y3_min],'-k','LineWidth',1)
hold off
xlabel('x (m)','FontSize',label_size)
ylabel('y (m)','FontSize',label_size)
title2 = sprintf('%.2f seconds',t(N));
title(title2,'FontSize',title_size)
set(gca,'LineWidth',1.2,'FontSize',tick_size)
legend(leg1,leg2,leg3,'Location','southwest')
axis equal
xlim([x2_min x2_max])
ylim([y2_min y2_max])

subplot(1,3,3)
N = 11;
plot(x1(1,1:N),x1(2,1:N),x2(1,1:N),x2(2,1:N),x3(1,1:N),x3(2,1:N),'LineWidth',2)
xlabel('x (m)','FontSize',label_size)
ylabel('y (m)','FontSize',label_size)
title3 = sprintf('%.2f seconds',t(N));
title(title3,'FontSize',title_size)
set(gca,'LineWidth',1.2,'FontSize',tick_size)
legend(leg1,leg2,leg3,'Location','southwest')
axis equal
xlim([x3_min x3_max])
ylim([y3_min y3_max])

%% Graph the error as a function of the change in distance and change in theta.

v = linspace(0,2.5,251);
omega = linspace(0,2.5,251);
dt = 3;
ds = v*dt;
dtheta = omega*dt;

err1 = NaN(251,251);
err2 = err1;
for i = 1:length(v)
    for j = 1:length(omega)
        x = [0 0 0 v(i) omega(j)];
        x1_new = odom1(dt,x);
        x2_new = odom2(dt,x);
        x3_new = odom3(dt,x);
        err1(j,i) = sqrt((x1_new(1)-x3_new(1))^2+(x1_new(2)-x3_new(2))^2);
        err2(j,i) = sqrt((x2_new(1)-x3_new(1))^2+(x2_new(2)-x3_new(2))^2);
    end
end

% y axis is distance traveled (v*dt), x axis is change in theta (omega*dt),
% z is distance of final (x,y) from that of odom3.
figure(2)
subplot(1,2,1)
surf(dtheta,ds,err1)
ylabel('\DeltaD (m)','FontSize',label_size)
xlabel('\Delta\theta (rad)','FontSize',label_size)
title('Error in A','FontSize',title_size)
set(gca,'LineWidth',1.2,'FontSize',tick_size)
shading interp
view(2)
axis equal
xlim([0 2.5*3])
ylim([0 2.5*3])
ax = gca;
ax.XTick = [0 pi/2 pi 3*pi/2 2*pi];
ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
colormap(hsv(256))
colorbar
caxis([0 10])

subplot(1,2,2)
surf(dtheta,ds,err2)
ylabel('\DeltaD (m)','FontSize',label_size)
xlabel('\Delta\theta (rad)','FontSize',label_size)
title('Error in B','FontSize',title_size)
set(gca,'LineWidth',1.2,'FontSize',tick_size)
shading interp
view(2)
axis equal
xlim([0 2.5*3])
ylim([0 2.5*3])
ax = gca;
ax.XTick = [0 pi/2 pi 3*pi/2 2*pi];
ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
colormap(hsv(256))
colorbar
caxis([0 10])

%% Calculate max error at max linear and angular velocity

vMax = 1.5; % m/s (Set in RT_Main*.vi on the cRIO)
omegaMax = 2.0; % rad/s (Set in RT_Main*.vi on the cRIO)
dtMax = 1/60.0; % Update rate of encoders
dt = linspace(0,dtMax,100);
% Initial conditions
xMax = [0 0 0 vMax omegaMax];
% Final conditions for all three algorithms
xMax1_new = odom1(dt,xMax);
xMax2_new = odom2(dt,xMax);
xMax3_new = odom3(dt,xMax);
% Compute difference in final location between 1st two algorithms and 3rd.
errMax1 = sqrt((xMax1_new(1,end)-xMax3_new(1,end))^2+(xMax1_new(2,end)-xMax3_new(2,end))^2);
errMax2 = sqrt((xMax2_new(1,end)-xMax3_new(1,end))^2+(xMax2_new(2,end)-xMax3_new(2,end))^2);
% Print to console
fprintf('Maximum error of algorithms 1 and 2 at vMax = %.2f, omegaMax = %.2f.\nError1 = %.8f\nError2 = %.8f\n',vMax,omegaMax,errMax1,errMax2);
% And plot
figure(3)
plot(xMax1_new(1,:),xMax1_new(2,:),xMax2_new(1,:),xMax2_new(2,:),xMax3_new(1,:),xMax3_new(2,:),'LineWidth',2)
xlabel('x (m)','FontSize',label_size)
ylabel('y (m)','FontSize',label_size)
title1 = sprintf('%.2f seconds',dtMax);
title(title1,'FontSize',title_size)
set(gca,'LineWidth',1.2,'FontSize',tick_size)
legend('odom1','odom2','odom3','Location','northeast')
axis equal
% xlim([x1_min x1_max])
% ylim([y1_min y1_max])