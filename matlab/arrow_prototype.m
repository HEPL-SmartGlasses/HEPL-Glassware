% Define the two points
xc = 0; yc = 0;

x0 = 1;
y0 = 1;

dx1 = 2;
dx2 = 3;

dy1 = 2;
dy2 = 1;

x1 = x0 - dx1; y1 = y0 + dy1;
x2 = x0 - dx1; y2 = y0 - dy1;
x3 = x0 - dx1 - dx2; y3 = y0 + dy2;
x4 = x0 - dx1 - dx2; y4 = y0 - dy2;
x5 = x0 - dx1; y5 = y0 + dy2;
x6 = x0 - dx1; y6 = y0 - dy2;

% Create a vector of x-coordinates and y-coordinates for the line
X = [x0 x1 x5 x3 x4 x6 x2 x0];
Y = [y0 y1 y5 y3 y4 y6 y2 y0];

% Rotate by theta
M = [X; Y];
theta = 0;
rot = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
Mc = [xc ; yc];

Mbar = rot * M + Mc;

Xbar = Mbar(1,:);
Ybar = Mbar(2,:);

close all;
figure()
% Plot the line using the plot() function
plot(Xbar, Ybar, 'b-', 'LineWidth', 2);

% Add labels and title to the plot
xlabel('x');
ylabel('y');
title('HEPL');