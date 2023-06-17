%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%  Code modified by Akshet Patel (Q3A)
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 20;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

%%
num_drones = 1;

time_interval = 0.02;
drone_traj = [];
%instantiate a drone object, input the axis and arena limits
drones = [];
for i = 1:num_drones
    drones = [drones Drone(ax1, spaceDim, num_drones, time_interval)];
end

drones(1).equ_inputs = solveInputs(zeros(1,3),drones(i));

while(drones(1).time < 150)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end

    if drones(1).operations_completed_successfully == true
        break
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    drone_traj = [drone_traj drones.pos];
    x = drone_traj(1, 1:width(drone_traj));
    y = drone_traj(2, 1:width(drone_traj));
    z = drone_traj(3, 1:width(drone_traj));
    plot3(ax1, x,y,z);
    % Define the points that the circular trajectory should pass through
    points = [2.5 2.5 5; 0 5 5; 2.5 7.5 5; 5 5 5];
    plot3(points(:,1), points(:,2), points(:,3), 'ro');

    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)

end

figure(2);
% plot position variation over time
subplot(3,1,1);
plot(drones.times,drones.xyzpos(1,:));
title('Variation of x (m) Coordinate Over Time (s)');
xlabel('Time (seconds)')
ylabel('x (m) Coordinate vs. Time (s)')
grid on;

subplot(3,1,2);
plot(drones.times,drones.xyzpos(2,:));
title('Variation of y (m) Coordinate Over Time (s)');
xlabel('Time (seconds)')
ylabel('y (m) Coordinate vs. Time (s)')
grid on;

subplot(3,1,3);
plot(drones.times,drones.xyzpos(3,:));
title('z (m) Coordinate vs. Time (s)');
xlabel('Time (seconds)')
ylabel('z: Distance (m) from origin')
grid on;

% plot orientation variation over time
figure(3);
subplot(3,1,1);
plot(drones.times,drones.orientation(1,:));
title('Variation of Roll Angle (°) vs. Time (s)');
xlabel('Time (seconds)')
ylabel('Roll Angle (°)')
grid on;

subplot(3,1,2);
plot(drones.times,drones.orientation(2,:));
title('Variation of Pitch Angle (°) vs. Time (s)');
xlabel('Time (seconds)')
ylabel('Pitch Angle (°)')
grid on;

subplot(3,1,3);
plot(drones.times,drones.orientation(3,:));
title('Variation Yaw Angle (°) vs. Time (s)');
xlabel('Time (seconds)')
ylabel('Yaw Angle (°)')
grid on;

figure

plot3(drones.xyzpos(1,:),drones.xyzpos(2,:),drones.xyzpos(3,:))
title('Quadcopter Trajectory (Q3a)')
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')
grid on;