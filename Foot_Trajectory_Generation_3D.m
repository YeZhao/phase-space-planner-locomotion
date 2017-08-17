%====================================
% Author:       Ye Zhao
% Date:         September 4th. 2016
% File Name:     Foot_Trajectory_Generation_3D.m
%====================================

% This file generate the smooth 3D foot trajectories based on the CoM data from Gait_Planner_3D.m

clear all;
% Font declaration
bigTextSize = 20;
mediumTextSize = 18;
textSize = 24;
smallTextSize = 14;
legendMargin = 0.3;

[s,w] = system('dir');
if s % for macintosh and unix
    addpath([pwd,'/func'])
else % for windows
    addpath([pwd,'\func'])
end

% Load the kinematic CoM Data generated from Gait_Planner_3D.m
pcom_struct = load('COM_AUTOMATIC_PLANE.dat');
vcom_struct = load('VCOM_AUTOMATIC_PLANE.dat');
feet_struct = load('FEET_AUTOMATIC_PLANE.dat');

tcom = pcom_struct(:,1);
pcom = pcom_struct(:,2:4);
vcom = vcom_struct(:,2:4);
tfeet = feet_struct(:,1);
feet = feet_struct(:,2:4);

% % Infer transition indexes
index_feet = [];

load HSTAIR_RAND.dat
for i = 1:length(HSTAIR_RAND) + 1
    if i == 1
        foo = find( feet(:,1) > feet(1,1) );
        index_feet(end+1) = foo(1); 
    else
        foo = find( feet(:,1) > feet(index_feet(end),1) );
        index_feet(end+1) = foo(1); 
    end
end
activefoot = '';
activefoot_index = 0; % this implies starting with left foot
dt_trajectory = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vertical feet initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%vmax = 2;%1.5;
tFinish_vertical_previous = 0;
tleft_vertical = [];
zleft_vertical = [];
dzleft_vertical = [];
ddzleft_vertical = [];
tright_vertical = [];
zright_vertical = [];
dzright_vertical = [];
ddzright_vertical = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% forward feet initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tFinish_forward_previous = 0;
tleft_forward = [];
xleft_forward = [];
dxleft_forward = [];
ddxleft_forward = [];
tright_forward = [];
xright_forward = [];
dxright_forward = [];
ddxright_forward = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lateral feet initializations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tFinish_lateral_previous = 0;
tleft_lateral = [];
yleft_lateral = [];
dyleft_lateral = [];
ddyleft_lateral = [];
tright_lateral = [];
yright_lateral = [];
dyright_lateral = [];
ddyright_lateral = [];

for i = 1:length(HSTAIR_RAND) + 2

disp('Foot Step');
disp(i);
vmax = 2.5;

if i == 1
    zStart(i) = feet(1,3);
elseif i == 2
    zStart(i) = zEnd(i-1);
else
    zStart(i) = feet(index_feet(i-2),3);
end
if i == length(HSTAIR_RAND) + 2
    zEnd(i) = feet(index_feet(i-1),3);
else
zEnd(i) = feet(index_feet(i),3);
end
height = max(zStart(i),zEnd(i)) + 0.15;

if i>=2 && zEnd(i) < zEnd(i-1) && zStart(i) < zEnd(i-1)
    vmax = 3.5;
    height = max(zStart(i),zEnd(i)) + 0.35;
end

if i == 1
tStart_vertical = tfeet(1);
else
tStart_vertical = tfeet(index_feet(i-1));
end
if i == length(HSTAIR_RAND) + 2
    tFinish_vertical = tStart_vertical + 0.8;%0.35;
else
tFinish_vertical = tfeet(index_feet(i));
end


if i == 1
    xleft_init_x = feet(1,1); 
    xright_init_x = feet(1,1);
end
if i == 1
    length_forward = feet(index_feet(1),1) - feet(1,1);
elseif i == 2
    length_forward = feet(index_feet(2),1) - feet(1,1);
elseif i == length(HSTAIR_RAND) + 2
    length_forward = feet(index_feet(i-1),1) - feet(index_feet(i-2),1);
else
    length_forward = feet(index_feet(i),1) - feet(index_feet(i-2),1);
end

tStart_forward = tStart_vertical;
tFinish_forward = tFinish_vertical;

if i == 1
    yleft_init_y = feet(1,2);
    yright_init_y = feet(1,2);
end
if i == 1
    length_lateral = feet(index_feet(1),2) + feet(1,2);
elseif i == 2
    length_lateral = feet(index_feet(2),2) - feet(1,2);
elseif i == length(HSTAIR_RAND) + 2
    length_lateral = -feet(index_feet(i-1),2) - feet(index_feet(i-2),2);
else
    length_lateral = feet(index_feet(i),2) - feet(index_feet(i-2),2);
end
tStart_lateral = tStart_vertical;
tFinish_lateral = tFinish_vertical;

% dealing with foot switching
if mod( activefoot_index, 2 ) == 0
    activefoot = 'left';
    if i>1
        x_init_forward = xleft_init_x;%feet(1,1);
        y_init_lateral = yleft_init_y;%feet(1,2);
    end
else
    activefoot = 'right';
    if i>1
        x_init_forward = xright_init_x;%feet(1,1);
        y_init_lateral = yright_init_y;%feet(1,2);
    end
end
activefoot_index = activefoot_index + 1;


% feet vertical trajectories
[tleft_vertical, zleft_vertical, dzleft_vertical, ddzleft_vertical,...
    tright_vertical, zright_vertical, dzright_vertical, ddzright_vertical] = ...
   feet_vertical_traj_func(activefoot, zStart(i), zEnd(i), height, tFinish_vertical_previous, ...
   tStart_vertical, tFinish_vertical, vmax,...
   dt_trajectory, tleft_vertical, zleft_vertical, dzleft_vertical, ddzleft_vertical,...
   tright_vertical, zright_vertical, dzright_vertical, ddzright_vertical);
tFinish_vertical_previous = tFinish_vertical;
 
% feet forward trajectories
if i == 1
    [tleft_forward, xleft_forward, dxleft_forward, ddxleft_forward,...
    tright_forward, xright_forward, dxright_forward, ddxright_forward] = ...
   feet_forward_traj_func(activefoot, xleft_init_x, length_forward,...
   tFinish_forward_previous, tStart_forward, tFinish_forward, dt_trajectory,...
   tleft_forward, xleft_forward, dxleft_forward, ddxleft_forward, tright_forward,...
   xright_forward,dxright_forward,ddxright_forward);
    tFinish_forward_previous = tFinish_forward;
else
    [tleft_forward, xleft_forward, dxleft_forward, ddxleft_forward,...
    tright_forward, xright_forward, dxright_forward, ddxright_forward] = ...
   feet_forward_traj_func(activefoot, x_init_forward, length_forward,...
   tFinish_forward_previous, tStart_forward, tFinish_forward, dt_trajectory,...
   tleft_forward, xleft_forward, dxleft_forward, ddxleft_forward, tright_forward,...
   xright_forward,dxright_forward,ddxright_forward);
tFinish_forward_previous = tFinish_forward;
end

if i == 1
% feet lateral trajectories
    [tleft_lateral, yleft_lateral, dyleft_lateral, ddyleft_lateral,...
    tright_lateral, yright_lateral, dyright_lateral, ddyright_lateral] = ...
   feet_lateral_traj_func(activefoot, yleft_init_y, length_lateral,...
   tFinish_lateral_previous, tStart_lateral, tFinish_lateral, dt_trajectory,...
   tleft_lateral, yleft_lateral, dyleft_lateral, ddyleft_lateral, tright_lateral,...
   yright_lateral,dyright_lateral,ddyright_lateral);
    tFinish_lateral_previous = tFinish_lateral;
else
    [tleft_lateral, yleft_lateral, dyleft_lateral, ddyleft_lateral,...
    tright_lateral, yright_lateral, dyright_lateral, ddyright_lateral] = ...
   feet_lateral_traj_func(activefoot, y_init_lateral, length_lateral,...
   tFinish_lateral_previous, tStart_lateral, tFinish_lateral, dt_trajectory,...
   tleft_lateral, yleft_lateral, dyleft_lateral, ddyleft_lateral, tright_lateral,...
   yright_lateral,dyright_lateral,ddyright_lateral);
    tFinish_lateral_previous = tFinish_lateral;
end
end
 
% process data to plot x-y-z com trajectory
min_dim = min(size(tright_forward,2),size(tright_vertical,2));
if size(tright_forward,2) < size(tright_vertical,2)
for ii = 1:size(tright_forward,2)
    index_foo = find(tright_forward(ii) < tfeet);
    pcom_xyz(ii,1:3) = [pcom(index_foo(1),1) pcom(index_foo(1),2) pcom(index_foo(1),3)];
end
else
    for ii = 1:size(tright_vertical,2)
    index_foo = find(tright_vertical(ii) < tfeet);
    pcom_xyz(ii,1:3) = [pcom(index_foo(1),1) pcom(index_foo(1),2) pcom(index_foo(1),3)];
end
end
 
% process data to plot xyz feet data
min_dim = min(size(tright_forward,2),size(tright_vertical,2));
xyz_right = zeros(min_dim,3);
xyz_left = zeros(min_dim,3);
if size(tright_forward,2) < size(tright_vertical,2)
    txyz = tright_forward;
    xyz_right(:,1) = xright_forward;
    xyz_left(:,1) = xleft_forward;
    xyz_right(:,2) = yright_lateral;
    xyz_left(:,2) = yleft_lateral;
    for ii = 1:size(tright_forward,2)
        index_time = find(tright_vertical >= tright_forward(ii));
        xyz_right(ii,3) = zright_vertical(index_time(1));
        xyz_left(ii,3) = zleft_vertical(index_time(1));
    end
else
    txyz = tright_vertical;
    xyz_right(:,3) = zright_vertical;
    xyz_left(:,3) = zleft_vertical;
    for ii = 1:size(tright_vertical,2)
        index_time = find(tright_forward >= tright_vertical(ii));
%         index_time(1)
        xyz_right(ii,1) = xright_forward(index_time(1));
        xyz_left(ii,1) = xleft_forward(index_time(1));
        xyz_right(ii,2) = yright_lateral(index_time(1));
        xyz_left(ii,2) = yleft_lateral(index_time(1));
    end
end

%%%%%%%%%%%%%%
% plotting
%%%%%%%%%%%%%%
orange = [1,3/5,0];

%%%%%%%%%%%%%vertical%%%%%%%%%%%%%%%%%
figure(1)
clf
plot(txyz, xyz_left(:,3),'linewidth',2.5);
hold on;
plot(txyz, xyz_right(:,3), 'r--','linewidth',2.5);
hold on;
plot(txyz, pcom_xyz(:,3),'color',orange,'linewidth',2.5);
% axis([0 6 0 1.7]);
axis([-1 7 -1.5 3.5]);
h_legend = legend('Left Leg','Right Leg','CoM','Location','NorthEast');
set(h_legend,'FontSize',mediumTextSize);
xlabel('t [s]','fontsize',mediumTextSize);
ylabel('z [m]','fontsize',mediumTextSize);
title('Vertical foot trajectories','fontsize',mediumTextSize);
set(gca,'fontsize',mediumTextSize);
box on;
grid on;
set(gcf,'PaperPositionMode','auto');
set(gcf,'PaperPosition',[0, 0, 7, 5]);
zoom on;
print -djpeg feet_vertical_trajectories

%%%%%%%%%%%%forward%%%%%%%%%%%%%%%%%
figure(2)
clf
plot(txyz, xyz_left(:,1),'linewidth',2.5);
hold on;
plot(txyz, xyz_right(:,1), 'r--','linewidth',2.5);
hold on;
plot(txyz, pcom_xyz(:,1),'color',orange,'linewidth',2.5);
axis([0 7 -0.5 6]);
h_legend = legend('Left Leg','Right Leg','CoM','Location','NorthWest');
set(h_legend,'FontSize',mediumTextSize);
xlabel('t [s]','fontsize',smallTextSize);
ylabel('x [m]','fontsize',smallTextSize);
title('Sagittal foot trajectories','fontsize',mediumTextSize);
set(gca,'fontsize',mediumTextSize);
box on;
grid on;
set(gcf,'PaperPositionMode','auto');
set(gcf,'PaperPosition',[0, 0, 7, 5]);
zoom on;
print -djpeg feet_forward_trajectories

%%%%%%%%%%%%%%%lateral%%%%%%%%%%%%%%%%%
figure(3)
clf
plot(txyz, xyz_left(:,2),'linewidth',2.5);
hold on;
plot(txyz, xyz_right(:,2), 'r--','linewidth',2.5);
hold on;
plot(txyz, pcom_xyz(:,2),'color',orange,'linewidth',2.5);
h_legend = legend('Left Leg','Right Leg','CoM','Location','NorthEast');
set(h_legend,'FontSize',mediumTextSize);
xlabel('t [s]','fontsize',smallTextSize);
ylabel('y [m]','fontsize',smallTextSize);
title('Lateral foot trajectories','fontsize',mediumTextSize);
axis([-0.5 7 -0.3 0.4]);
set(gca,'fontsize',mediumTextSize);
box on;
grid on;
set(gcf,'PaperPositionMode','auto');
set(gcf,'PaperPosition',[0, 0, 7, 5]);
zoom on;
print -djpeg feet_lateral_trajectories

figure(4)
clf
plot(xyz_left(:,1),xyz_left(:,3),'linewidth',2.5);
hold on;
plot(xyz_right(:,1),xyz_right(:,3),'r--','linewidth',2.5);
plot(pcom_xyz(:,1),pcom_xyz(:,3),'color',orange,'linewidth',2.5);
h_legend = legend('Left Leg','Right Leg','CoM','Location','NorthEast');
set(h_legend,'FontSize',mediumTextSize);
xlabel('x [m]','fontsize',smallTextSize);
ylabel('z [m]','fontsize',smallTextSize);
title('Feet and CoM Sagittal geometric trajectories','fontsize',mediumTextSize);
% axis([-0.5 4 0 1.6]);
axis([-1 5 -1.5 3.5]);
set(gca,'fontsize',smallTextSize);
box on;
grid on;
set(gcf,'PaperPositionMode','auto');
set(gcf,'PaperPosition',[0, 0, 7, 5]);
zoom on;
print -djpeg feet_sagittal_geometric_trajectories

figure(5)
plot(pcom_xyz(:,1),pcom_xyz(:,2),'linewidth',2.5);
xlabel('x [m]');
ylabel('y [m]');
axis([0 5 -0.3 0.4]);

save THREED_LEFT_FEET_TRAJECTORY.dat xyz_left -ASCII
save THREED_RIGHT_FEET_TRAJECTORY.dat xyz_right -ASCII
save THREED_COM_TRAJECTORY.dat pcom_xyz -ASCII
save THREED_TIME.dat txyz -ASCII
%make_movie_after_draw_robot_automatic;