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
legend('odom1','odom2','odom3','Location','northeast')
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
legend('odom1','odom2','odom3','Location','southwest')
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
legend('odom1','odom2','odom3','Location','southwest')
axis equal
xlim([x3_min x3_max])
ylim([y3_min y3_max])
