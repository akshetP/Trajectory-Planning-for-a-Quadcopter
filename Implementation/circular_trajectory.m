%%
clc;
clear;
close all;
% Define the points that the circular trajectory should pass through
points = [2.5 2.5 5; 0 5 5; 2.5 7.5 5; 5 5 5];

% Define the center and radius of the circle
center = [2.5 5 5];
radius = 2.5;
z = 5.0;

% Define the start and end angles of the circular trajectory
startAngle = 0;
endAngle = 2*pi;

% Use linspace to generate a set of evenly spaced points along the circumference of the circle
angles = linspace(startAngle, endAngle, 20);

% Compute the x and y coordinates of the points on the circumference
x = flip(center(1) + radius*cos(angles));
y = flip(center(2) + radius*sin(angles));
z = z*ones(size(x));
coordinates = vertcat(x, y, z);
newPoint = [5; 5; 0];
coordinates = cat(2, coordinates, newPoint);
disp(coordinates)
% Create a 3D plot with the points on the circumference
plot3(x, y, z, '-o');
hold on
plot3(points(:,1), points(:,2), points(:,3), 'ro');
grid on;