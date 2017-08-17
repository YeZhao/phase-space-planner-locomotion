%====================================
% Author:       Ye Zhao
% Date:         September 4th. 2016
% File Name:     Animation_3D_Locomotion.m
%====================================

% This file generate the 3D dynamic walking, viewed from several different angles.
% The leg configuration corresponds to Hume Dimension.
% The 3D knee data is generated by inverse kinematics.

clf;
% these four DAT files are generated from Foot_Trajectory_Generation_3D.m
load THREED_COM_TRAJECTORY.dat
load THREED_LEFT_FEET_TRAJECTORY.dat
load THREED_RIGHT_FEET_TRAJECTORY.dat
load THREED_TIME.dat

% parameter declaration
pcom = THREED_COM_TRAJECTORY;
% adjustment for hume configuration
pcom(:,3) = pcom(:,3) - 0.3;
feet_left = THREED_LEFT_FEET_TRAJECTORY;
feet_right = THREED_RIGHT_FEET_TRAJECTORY;
time = THREED_TIME;

x_left_foot = feet_left(:,1);
y_left_foot = feet_left(:,2);
z_left_foot = feet_left(:,3);

x_right_foot = feet_right(:,1);
y_right_foot = feet_right(:,2);
z_right_foot = feet_right(:,3);

% make the movies
skipp = 15;
foo = diff(time);
framerate = round(1/foo(1)) / skipp;
framerate = round(framerate);
% quicktime parameters
% MakeQTMovie('start','Dynamic Walking.mov');
% % MakeQTMovie('start','3D Dynamic Walking.mov');
% % MakeQTMovie('start','Side View Walking.mov');
% % MakeQTMovie('start','Top View Walking.mov');
% % MakeQTMovie('start','Front View Walking.mov');
% 
% MakeQTMovie('quality',0.75);
% MakeQTMovie('size',[1280 680]);
% MakeQTMovie('framerate',framerate);

% initialize legs. Here we use the hume leg configuration
L1 = 0.55;
L2 = 0.45;
L_torso = 0.6;
L2_compensate = 0.5;

% The knee trajectory generation
for ii = 1:size(time,2)
%%%%%%%%%%%%%%%%%%%%%%%%% left foot%%%%%%%%%%%%%%%%%%%%%%%%
    L = sqrt( (pcom(ii,1) - x_left_foot(ii))^2 + (pcom(ii,2) - y_left_foot(ii))^2 + ...
        (pcom(ii,3) - z_left_foot(ii))^2 );
    bb = (L1^2 - L2^2 + L^2) / (2 * L);
    aa = sqrt( L1^2 - bb^2);
    theta = atan(bb/aa);
    theta_rot = atan2(sqrt((pcom(ii,3) - z_left_foot(ii))^2 + (pcom(ii,2) - y_left_foot(ii))^2), pcom(ii,1) - x_left_foot(ii));
    % Rotation matrix to convert the local leg configuration into the global frame
    Roty = [cos(theta_rot) 0 sin(theta_rot);0 1 0;-sin(theta_rot) 0 cos(theta_rot)];
    fai = atan2(abs(pcom(ii,2)  - y_left_foot(ii)), abs(-pcom(ii,3)  + z_left_foot(ii)));
    Rotx = [1 0 0;0 cos(fai) -sin(fai);0 sin(fai) cos(fai)];
    xzk_left(: , ii) = Rotx * Roty' * [-L1 * sin(theta); 0; -L1 * cos(theta)]; 
    %knee traj
    x_left_knee(ii) = pcom(ii,1) + xzk_left(1,ii);
    y_left_knee(ii) = pcom(ii,2) + xzk_left(2,ii);
    z_left_knee(ii) = pcom(ii,3) + xzk_left(3,ii);

%%%%%%%%%%%%%%%%%%%%%%%%% right foot%%%%%%%%%%%%%%%%%%%%%%%%
    L = sqrt( (pcom(ii,1) - x_right_foot(ii))^2 + (pcom(ii,2) - y_right_foot(ii))^2 + ...
        (pcom(ii,3) - z_right_foot(ii))^2 );
    bb = (L1^2 - L2^2 + L^2) / (2 * L);
    aa = sqrt( L1^2 - bb^2);
    theta = atan( bb/aa);
    theta_rot = atan2(sqrt((pcom(ii,3) - z_right_foot(ii))^2 + (pcom(ii,2) - y_right_foot(ii))^2), pcom(ii,1) - x_right_foot(ii));
    % Rotation matrix to convert the local leg configuration into the global frame
    Roty = [cos(theta_rot) 0 sin(theta_rot);0 1 0;-sin(theta_rot) 0 cos(theta_rot)];
    fai = -atan2(abs(pcom(ii,2)  - y_right_foot(ii)), abs(-pcom(ii,3)  + z_right_foot(ii)));
    Rotx = [1 0 0;0 cos(fai) -sin(fai);0 sin(fai) cos(fai)];
    xzk_right(: , ii) = Rotx * Roty' * [-L1 * sin(theta); 0; -L1 * cos(theta)]; 
    %knee traj
    x_right_knee(ii) = pcom(ii,1) + xzk_right(1,ii);
    y_right_knee(ii) = pcom(ii,2) + xzk_right(2,ii);
    z_right_knee(ii) = pcom(ii,3) + xzk_right(3,ii);
end

x_left_knee = real(x_left_knee);
y_left_knee = real(y_left_knee);
z_left_knee = real(z_left_knee);

x_right_knee = real(x_right_knee);
y_right_knee = real(y_right_knee);
z_right_knee = real(z_right_knee);

%%%%%%%%%%Disributed Structure%%%%%%%%%%%%
y_left_knee = y_left_knee + 0.28;
y_right_knee = y_right_knee - 0.28;

x_left_hip = pcom(:,1);
y_left_hip = pcom(:,2) + 0.28;
z_left_hip = pcom(:,3);

x_right_hip = pcom(:,1);
y_right_hip = pcom(:,2) - 0.28;
z_right_hip = pcom(:,3);
%%%%%%%%%%Disributed Structure%%%%%%%%%%%%

x_left_knee = x_left_knee';
y_left_knee = y_left_knee';
z_left_knee = z_left_knee';

x_right_knee = x_right_knee';
y_right_knee = y_right_knee';
z_right_knee = z_right_knee';

%Matlab Animation Generation

%Font Declaration
bigTextSize = 20;
mediumTextSize = 18;
textSize = 24;
smallTextSize = 14;
legendMargin = 0.3;

%3D Animation Initialization

% stair initial edge tiny tuning
edge_tune = 0.09;

% draw link ellipsoid
leg_semi_radius = 0.07;%leg link ellipsoid radius

[xx_torso, yy_torso, zz_torso] = ellipsoid((pcom(1,1) + pcom(1,1) + 0.03)/2,pcom(1,2),pcom(1,3) + L_torso/2,leg_semi_radius,L_torso/2,leg_semi_radius,30);
[xx_hip, yy_hip, zz_hip] = ellipsoid((x_left_hip(1)+x_right_hip(1))/2,(y_left_hip(1)+y_right_hip(1))/2,(z_left_hip(1)+z_right_hip(1))/2,leg_semi_radius,0.28,leg_semi_radius,30);
[xx_leftthigh, yy_leftthigh, zz_leftthigh] = ellipsoid((x_left_knee(1)+x_left_hip(1))/2,(y_left_knee(1)+y_left_hip(1))/2,(z_left_knee(1)+z_left_hip(1))/2,L1/2,leg_semi_radius,leg_semi_radius,30);
[xx_leftcalf, yy_leftcalf, zz_leftcalf] = ellipsoid((x_left_foot(1)+x_left_knee(1))/2,(y_left_foot(1)+y_left_knee(1))/2,(z_left_foot(1)+z_left_knee(1))/2,L2/2,leg_semi_radius,leg_semi_radius,30);
[xx_rightthigh, yy_rightthigh, zz_rightthigh] = ellipsoid((x_right_knee(1)+x_right_hip(1))/2,(y_right_knee(1)+y_right_hip(1))/2,(z_right_knee(1)+z_right_hip(1))/2,L1/2,leg_semi_radius,leg_semi_radius,30);
[xx_rightcalf, yy_rightcalf, zz_rightcalf] = ellipsoid((x_right_foot(1)+x_right_knee(1))/2,(y_right_foot(1)+y_left_knee(1))/2,(z_left_foot(1)+z_right_knee(1))/2,L2/2,leg_semi_radius,leg_semi_radius,30);

left_leg_color = [160 160 160]/255;%[160 160 160]
joint_color = [47,79,79]/255;
hip_color = joint_color;%[1,0,0];
% left_leg_color = [0.1333,0.5451,0.1333];
% left_leg_color = [0.1843    0.3098    0.3098];
% colormap([hip_color; left_leg_color]);
colormap(left_leg_color)

%%%%%%%%%%%%%%%%%%%subplot1%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,2,[1 3]);
link101mesh = mesh(xx_torso, yy_torso, zz_torso);
hold on;
link10mesh = mesh(xx_hip, yy_hip, zz_hip, 'Facecolor', 'flat');%, 'Facecolor', 'flat','FaceAlpha',0
hold on;
link11mesh = mesh(xx_leftcalf, yy_leftcalf, zz_leftcalf, 'Facecolor', 'flat');
hold on;
link12mesh = mesh(xx_rightcalf, yy_rightcalf, zz_rightcalf, 'Facecolor', 'flat');
hold on;
link13mesh = mesh(xx_leftthigh, yy_leftthigh, zz_leftthigh, 'Facecolor', 'flat');
hold on;
link14mesh = mesh(xx_rightthigh, yy_rightthigh, zz_rightthigh, 'Facecolor', 'flat');
hold on;

% set(gcf,'position',[0, 0, 15, 10]);
% figure('Position',[10 10 10 10]);
h11=line('color', hip_color, 'marker', '.', 'markersize', 40);
link101=line([pcom(1,1) pcom(1,1) + 0.03],[pcom(1,2) pcom(1,2)],[pcom(1,3) pcom(1,3) + L_torso],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link10=line([x_left_hip(1),x_right_hip(1)],[y_left_hip(1),y_right_hip(1)],[z_left_hip(1),z_right_hip(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link11=line([x_left_foot(1),x_left_foot(1)],[y_left_foot(1),y_left_foot(1)],[z_left_foot(1),z_left_foot(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link12=line([x_right_foot(1),x_right_foot(1)],[y_right_foot(1),y_right_foot(1)],[z_right_foot(1),z_right_foot(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link13=line([x_left_knee(1),x_left_knee(1)],[y_left_knee(1),y_left_knee(1)],[z_left_knee(1),z_left_knee(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link14=line([x_right_knee(1),x_right_knee(1)],[y_right_knee(1),y_right_knee(1)],[z_right_knee(1),z_right_knee(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
grid on;

%Draw the stairs
load FOOT_CONTACT_X.dat
foot_contact_x = FOOT_CONTACT_X;
Hstair_rand = load('HSTAIR_RAND.dat');

%flat floor
x_i = foot_contact_x(1) -0.2;
y_i = -1.05;
z_i = 0.05;
lengthx1 = (foot_contact_x(3) + foot_contact_x(2))/2 - x_i + edge_tune - 0.05;
lengthy1 = 2.1;
lengthz1 = -0.2;

terrain_righttop_color = [128,0,0]/255;
terrain_lefttop_color = [178,34,34]/255;
terrain_bottom_color = [128,0,0]/255;
terrain_front_color = [139,0,0]/255;
terrain_back_color = [128,0,0]/255;
terrain_rightside_color = [165,42,42]/255;
terrain_leftside_color = [128,0,0]/255;

z_center_change = 0.2;
z_bottom_change = 0.2 + 0.1;

% % convex terrain
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_righttop_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i+lengthy1 y_i+lengthy1 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_lefttop_color);
patch([x_i+lengthx1 x_i+lengthx1 x_i+lengthx1 x_i+lengthx1 x_i+lengthx1],[y_i 0 y_i+lengthy1 y_i+lengthy1 y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_front_color);
patch([x_i x_i x_i x_i x_i],[y_i 0 y_i+lengthy1 y_i+lengthy1 y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_back_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i y_i y_i],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_rightside_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i+lengthy1 y_i+lengthy1 y_i+lengthy1 y_i+lengthy1],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_leftside_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i y_i+lengthy1 y_i+lengthy1],[z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change],terrain_bottom_color);


for i = 1: length(Hstair_rand)
    %first stair
    if i == 1
        x_init(i) = (foot_contact_x(i+2) + foot_contact_x(i+1))/2 + edge_tune;
    else
        x_init(i) = x_init(i-1) + lengthx(i-1);
    end
    
    y_i = -1.05;
    z_i = z_i + Hstair_rand(i);
    
    if i == length(Hstair_rand)
        lengthx(i) = lengthx(i-1) + 0.05;
    elseif i == length(Hstair_rand) - 1
        lengthx(i) = (foot_contact_x(i+4) - foot_contact_x(i+2))/2;% + 0.08;
    elseif i == 1
        lengthx(i) = (foot_contact_x(i+3) - foot_contact_x(i+1))/2;%+ 0.05
    else
        lengthx(i) = (foot_contact_x(i+3) - foot_contact_x(i+1))/2;
    end
    
    lengthy(i) = 2.1;%1.1;
    lengthz(i) = -0.2;
    
    % % convex terrain
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_righttop_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i+lengthy(i) y_i+lengthy(i) 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_lefttop_color);
    patch([x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i)],[y_i 0 y_i+lengthy(i) y_i+lengthy(i) y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_front_color);
    patch([x_init(i) x_init(i) x_init(i) x_init(i) x_init(i)],[y_i 0 y_i+lengthy(i) y_i+lengthy(i) y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_back_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i y_i y_i],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_rightside_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i+lengthy(i) y_i+lengthy(i) y_i+lengthy(i) y_i+lengthy(i)],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_leftside_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i y_i+lengthy(i) y_i+lengthy(i)],[z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change],terrain_bottom_color);

end
% axis('position', [0, 0, 15, 10]);
% set(gcf,'PaperPosition',[0, 10, 5, 15]);
% h111 = subplot(2,2,1);
% set(h111, 'position',[15 20 5 15]);
%     subplot('Position',[0.1, 0.6, 0.3, 0.3]);
%%%%%%%%%%%%%%%%%%%subplot2%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,2,2);
hold on;
link201mesh = mesh(xx_torso, yy_torso, zz_torso, 'Facecolor', 'flat');
hold on;
link20mesh = mesh(xx_hip, yy_hip, zz_hip, 'Facecolor', 'flat');
hold on;
link22mesh = mesh(xx_leftthigh, yy_leftthigh, zz_leftthigh, 'Facecolor', 'flat');
hold on;
link21mesh = mesh(xx_leftcalf, yy_leftcalf, zz_leftcalf, 'Facecolor', 'flat');
hold on;
link23mesh = mesh(xx_rightthigh, yy_rightthigh, zz_rightthigh, 'Facecolor', 'flat');
hold on;
link24mesh = mesh(xx_rightcalf, yy_rightcalf, zz_rightcalf, 'Facecolor', 'flat');
hold on;

h21=line('color', hip_color, 'marker', '.', 'markersize', 40);
link201=line([pcom(1,1) pcom(1,1) + 0.03],[pcom(1,2) pcom(1,2)],[pcom(1,3) pcom(1,3) + L_torso],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',25);
link20=line([x_left_hip(1),x_right_hip(1)],[y_left_hip(1),y_right_hip(1)],[z_left_hip(1),z_right_hip(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',25);
link21=line([x_left_foot(1),x_left_foot(1)],[y_left_foot(1),y_left_foot(1)],[z_left_foot(1),z_left_foot(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link22=line([x_right_foot(1),x_right_foot(1)],[y_right_foot(1),y_right_foot(1)],[z_right_foot(1),z_right_foot(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link23=line([x_left_knee(1),x_left_knee(1)],[y_left_knee(1),y_left_knee(1)],[z_left_knee(1),z_left_knee(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link24=line([x_right_knee(1),x_right_knee(1)],[y_right_knee(1),y_right_knee(1)],[z_right_knee(1),z_right_knee(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
grid on;

%Draw the stairs

%flat floor
x_i = foot_contact_x(1) -0.2;
y_i = -1.05;
z_i = 0;
lengthx1 = (foot_contact_x(3) + foot_contact_x(2))/2 - x_i + edge_tune;
lengthy1 = 2.1;
lengthz1 = -0.2;

% % convex terrain
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_righttop_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i+lengthy1 y_i+lengthy1 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_lefttop_color);
patch([x_i+lengthx1 x_i+lengthx1 x_i+lengthx1 x_i+lengthx1 x_i+lengthx1],[y_i 0 y_i+lengthy1 y_i+lengthy1 y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_front_color);
patch([x_i x_i x_i x_i x_i],[y_i 0 y_i+lengthy1 y_i+lengthy1 y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_back_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i y_i y_i],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_rightside_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i+lengthy1 y_i+lengthy1 y_i+lengthy1 y_i+lengthy1],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_leftside_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i y_i+lengthy1 y_i+lengthy1],[z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change],terrain_bottom_color);

for i = 1: length(Hstair_rand)
    %first stair
    if i == 1
        x_init(i) = (foot_contact_x(i+2) + foot_contact_x(i+1))/2 + edge_tune;
    else
        x_init(i) = x_init(i-1) + lengthx(i-1);
    end
    
    y_i = -1.05;
    z_i = z_i + Hstair_rand(i);
    
    if i == length(Hstair_rand)
        lengthx(i) = lengthx(i-1);
    elseif i == length(Hstair_rand) - 1
        lengthx(i) = (foot_contact_x(i+4) - foot_contact_x(i+2))/2 + 0.25;
    elseif i == 1
        lengthx(i) = (foot_contact_x(i+3) - foot_contact_x(i+1))/2;
    else
        lengthx(i) = (foot_contact_x(i+3) - foot_contact_x(i+1))/2;
    end
    lengthy(i) = 2.1;
    lengthz(i) = -0.2;
    
    % % convex terrain
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_righttop_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i+lengthy(i) y_i+lengthy(i) 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_lefttop_color);
    patch([x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i)],[y_i 0 y_i+lengthy(i) y_i+lengthy(i) y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_front_color);
    patch([x_init(i) x_init(i) x_init(i) x_init(i) x_init(i)],[y_i 0 y_i+lengthy(i) y_i+lengthy(i) y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_back_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i y_i y_i],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_rightside_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i+lengthy(i) y_i+lengthy(i) y_i+lengthy(i) y_i+lengthy(i)],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_leftside_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i y_i+lengthy(i) y_i+lengthy(i)],[z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change],terrain_bottom_color);

end
%%%%%%%%%%%%%%%%%%%subplot3%%%%%%%%%%%%%%%%%%%%%%%%
subplot(2,2,4);
link301mesh = mesh(xx_torso, yy_torso, zz_torso, 'Facecolor', 'flat');
hold on;
link30mesh = mesh(xx_hip, yy_hip, zz_hip, 'Facecolor', 'flat');
hold on;
link32mesh = mesh(xx_leftthigh, yy_leftthigh, zz_leftthigh, 'Facecolor', 'flat');
hold on;
link31mesh = mesh(xx_leftcalf, yy_leftcalf, zz_leftcalf, 'Facecolor', 'flat');
hold on;
link33mesh = mesh(xx_rightthigh, yy_rightthigh, zz_rightthigh, 'Facecolor', 'flat');
hold on;
link34mesh = mesh(xx_rightcalf, yy_rightcalf, zz_rightcalf, 'Facecolor', 'flat');
hold on;

h31=line('color', hip_color, 'marker', '.', 'markersize', 40);
link301=line([pcom(1,1) pcom(1,1) + 0.03],[pcom(1,2) pcom(1,2)],[pcom(1,3) pcom(1,3) + L_torso],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',25);
link30=line([x_left_hip(1),x_right_hip(1)],[y_left_hip(1),y_right_hip(1)],[z_left_hip(1),z_right_hip(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',25);
link31=line([x_left_foot(1),x_left_foot(1)],[y_left_foot(1),y_left_foot(1)],[z_left_foot(1),z_left_foot(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link32=line([x_right_foot(1),x_right_foot(1)],[y_right_foot(1),y_right_foot(1)],[z_right_foot(1),z_right_foot(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link33=line([x_left_knee(1),x_left_knee(1)],[y_left_knee(1),y_left_knee(1)],[z_left_knee(1),z_left_knee(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
link34=line([x_right_knee(1),x_right_knee(1)],[y_right_knee(1),y_right_knee(1)],[z_right_knee(1),z_right_knee(1)],'color',joint_color,'Linewidth',0.001,'Marker','.','MarkerSize',30);
grid on;

%Draw the stairs

%flat floor
x_i = foot_contact_x(1) -0.2;
y_i = -1.05;%-0.55;
z_i = 0;
lengthx1 = (foot_contact_x(3) + foot_contact_x(2))/2 - x_i + edge_tune;
lengthy1 = 2.1;
lengthz1 = -0.2;

% % convex terrain
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_righttop_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i+lengthy1 y_i+lengthy1 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_lefttop_color);
patch([x_i+lengthx1 x_i+lengthx1 x_i+lengthx1 x_i+lengthx1 x_i+lengthx1],[y_i 0 y_i+lengthy1 y_i+lengthy1 y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_front_color);
patch([x_i x_i x_i x_i x_i],[y_i 0 y_i+lengthy1 y_i+lengthy1 y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_back_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i y_i y_i],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_rightside_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i+lengthy1 y_i+lengthy1 y_i+lengthy1 y_i+lengthy1],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_leftside_color);
patch([x_i x_i+lengthx1 x_i+lengthx1 x_i],[y_i y_i y_i+lengthy1 y_i+lengthy1],[z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change],terrain_bottom_color);

for i = 1: length(Hstair_rand)
    %first stair
    %first stair
    if i == 1
        x_init(i) = (foot_contact_x(i+2) + foot_contact_x(i+1))/2 + edge_tune;
    else
        x_init(i) = x_init(i-1) + lengthx(i-1);
    end
    
    y_i = -1.05;
    z_i = z_i + Hstair_rand(i);
    
    if i == length(Hstair_rand)
        lengthx(i) = lengthx(i-1);
    elseif i == length(Hstair_rand) - 1
        lengthx(i) = (foot_contact_x(i+4) - foot_contact_x(i+2))/2 + 0.25;
    elseif i == 1
        lengthx(i) = (foot_contact_x(i+3) - foot_contact_x(i+1))/2;
    else
        lengthx(i) = (foot_contact_x(i+3) - foot_contact_x(i+1))/2;
    end
    lengthy(i) = 2.1;
    lengthz(i) = -0.2;
    
    % % convex terrain
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_righttop_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i+lengthy(i) y_i+lengthy(i) 0 0],[z_i, z_i, z_i - z_center_change, z_i - z_center_change],terrain_lefttop_color);
    patch([x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i)],[y_i 0 y_i+lengthy(i) y_i+lengthy(i) y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_front_color);
    patch([x_init(i) x_init(i) x_init(i) x_init(i) x_init(i)],[y_i 0 y_i+lengthy(i) y_i+lengthy(i) y_i],[z_i, z_i - z_center_change, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_back_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i y_i y_i],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_rightside_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i+lengthy(i) y_i+lengthy(i) y_i+lengthy(i) y_i+lengthy(i)],[z_i, z_i, z_i - z_bottom_change, z_i - z_bottom_change],terrain_leftside_color);
    patch([x_init(i) x_init(i)+lengthx(i) x_init(i)+lengthx(i) x_init(i)],[y_i y_i y_i+lengthy(i) y_i+lengthy(i)],[z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change, z_i - z_bottom_change],terrain_bottom_color);

end

ii = 1;
% 3D Animation Process. Draw the current step while erase the previous step
for j=skipp+1:skipp:size(time,2)

    torso_ellip_center = [(pcom(j,1) + pcom(j,1) + 0.03)/2,pcom(j,2),pcom(j,3) + L_torso/2];
    [xx_torso, yy_torso, zz_torso] = ellipsoid(torso_ellip_center(1),torso_ellip_center(2),torso_ellip_center(3),L_torso/2,leg_semi_radius,leg_semi_radius,30);
    torso_ellip_vector = [-0.03,0,-L_torso/2];
    torso_ellip_vector = torso_ellip_vector/norm(torso_ellip_center,2);
    torso_refer_vector = [1,0,0];
    torso_rotate = vrrotvec(torso_refer_vector,torso_ellip_vector) * 180/pi;%degree
    
    hip_ellip_center = [(x_left_hip(j)+x_right_hip(j))/2,(y_left_hip(j)+y_right_hip(j))/2,(z_left_hip(j)+z_right_hip(j))/2];
    [xx_hip, yy_hip, zz_hip] = ellipsoid(hip_ellip_center(1),hip_ellip_center(2),hip_ellip_center(3),leg_semi_radius,0.28,leg_semi_radius,30);
    hip_ellip_vector = [x_left_hip(j)-x_right_hip(j), y_left_hip(j)-y_right_hip(j), z_left_hip(j)-z_right_hip(j)];
    
    leftthigh_ellip_center = [(x_left_knee(j)+x_left_hip(j))/2,(y_left_knee(j)+y_left_hip(j))/2,(z_left_knee(j)+z_left_hip(j))/2];
    [xx_leftthigh, yy_leftthigh, zz_leftthigh] = ellipsoid(leftthigh_ellip_center(1),leftthigh_ellip_center(2),leftthigh_ellip_center(3),L1/2,leg_semi_radius,leg_semi_radius,30);
    leftthigh_ellip_vector = [x_left_knee(j)-x_left_hip(j),y_left_knee(j)-y_left_hip(j),z_left_knee(j)-z_left_hip(j)];
    leftthigh_ellip_vector = leftthigh_ellip_vector/norm(leftthigh_ellip_vector,2);
    leftthigh_refer_vector = [1,0,0];
    leftthigh_rotate = vrrotvec(leftthigh_refer_vector,leftthigh_ellip_vector) * 180/pi;%degree
    
    leftcalf_ellip_center = [(x_left_foot(j)+x_left_knee(j))/2,(y_left_foot(j)+y_left_knee(j))/2,(z_left_foot(j)+z_left_knee(j))/2];
    [xx_leftcalf, yy_leftcalf, zz_leftcalf] = ellipsoid(leftcalf_ellip_center(1),leftcalf_ellip_center(2),leftcalf_ellip_center(3),L2_compensate/2,leg_semi_radius,leg_semi_radius,30);
    leftcalf_ellip_vector = [x_left_foot(j)-x_left_knee(j),y_left_foot(j)-y_left_knee(j),z_left_foot(j)-z_left_knee(j)];
    leftcalf_ellip_vector = leftcalf_ellip_vector/norm(leftcalf_ellip_vector,2);
    leftcalf_refer_vector = [1,0,0];
    leftcalf_rotate = vrrotvec(leftcalf_refer_vector,leftcalf_ellip_vector) * 180/pi;%degree

    rightthigh_ellip_center = [(x_right_knee(j)+x_right_hip(j))/2,(y_right_knee(j)+y_right_hip(j))/2,(z_right_knee(j)+z_right_hip(j))/2];
    [xx_rightthigh, yy_rightthigh, zz_rightthigh] = ellipsoid(rightthigh_ellip_center(1),rightthigh_ellip_center(2),rightthigh_ellip_center(3),L1/2,leg_semi_radius,leg_semi_radius,30);
    rightthigh_ellip_vector = [x_right_knee(j)-x_right_hip(j),y_right_knee(j)-y_right_hip(j),z_right_knee(j)-z_right_hip(j)];
    rightthigh_ellip_vector = rightthigh_ellip_vector/norm(rightthigh_ellip_vector,2);
    rightthigh_refer_vector = [1,0,0];
    rightthigh_rotate = vrrotvec(rightthigh_refer_vector,rightthigh_ellip_vector) * 180/pi;%degree
    
    rightcalf_ellip_center = [(x_right_foot(j)+x_right_knee(j))/2,(y_right_foot(j)+y_right_knee(j))/2,(z_right_foot(j)+z_right_knee(j))/2];
    [xx_rightcalf, yy_rightcalf, zz_rightcalf] = ellipsoid(rightcalf_ellip_center(1),rightcalf_ellip_center(2),rightcalf_ellip_center(3),L2_compensate/2,leg_semi_radius,leg_semi_radius,30);
    rightcalf_ellip_vector = [x_right_foot(j)-x_right_knee(j),y_right_foot(j)-y_right_knee(j),z_right_foot(j)-z_right_knee(j)];
    rightcalf_ellip_vector = rightcalf_ellip_vector/norm(rightcalf_ellip_vector,2);
    rightcalf_refer_vector = [1,0,0];
    rightcalf_rotate = vrrotvec(rightcalf_refer_vector,rightcalf_ellip_vector) * 180/pi;%degree

    %%%%%%%%%%%%%%%%%%%subplot1%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,2,[1 3]);

    set(link101mesh,'XData',xx_torso,'YData',yy_torso,'ZData',zz_torso, 'Facecolor', 'flat');
    rotate(link101mesh,[torso_rotate(1:3)],torso_rotate(4),torso_ellip_center);
    hold on; 
    set(link10mesh,'XData',xx_hip,'YData',yy_hip,'ZData',zz_hip, 'Facecolor', 'flat');
    hold on; 
    set(link13mesh,'XData',xx_leftthigh,'YData',yy_leftthigh,'ZData',zz_leftthigh, 'Facecolor', 'flat');
    rotate(link13mesh,[leftthigh_rotate(1:3)],leftthigh_rotate(4),leftthigh_ellip_center);
    hold on; 
    set(link11mesh,'XData',xx_leftcalf,'YData',yy_leftcalf,'ZData',zz_leftcalf, 'Facecolor', 'flat');
    rotate(link11mesh,[leftcalf_rotate(1:3)],leftcalf_rotate(4),leftcalf_ellip_center);
    hold on; 
    set(link12mesh,'XData',xx_rightcalf,'YData',yy_rightcalf,'ZData',zz_rightcalf, 'Facecolor', 'flat');
    rotate(link12mesh,[rightcalf_rotate(1:3)],rightcalf_rotate(4),rightcalf_ellip_center);
    hold on; 
    set(link14mesh,'XData',xx_rightthigh,'YData',yy_rightthigh,'ZData',zz_rightthigh, 'Facecolor', 'flat');
    rotate(link14mesh,[rightthigh_rotate(1:3)],rightthigh_rotate(4),rightthigh_ellip_center);
    hold on; 
    
    patch([pcom(j-skipp,1) pcom(j,1)],[pcom(j-skipp,2) pcom(j,2)],[pcom(j-skipp,3) pcom(j,3)],'c');
    set(link101,'XData',[pcom(j,1) pcom(j,1) + 0.03],'YData',[pcom(j,2) pcom(j,2)],'ZData',[pcom(j,3) pcom(j,3) + L_torso]);%ptorso_tip is defined above
    set(link10,'XData',[x_left_hip(j),x_right_hip(j)],'YData',[y_left_hip(j),y_right_hip(j)],'ZData',[z_left_hip(j),z_right_hip(j)]);
    set(link11,'XData',[x_left_foot(j),x_left_foot(j)],'YData',[y_left_foot(j),y_left_foot(j)],'ZData',[z_left_foot(j),z_left_foot(j)]); 
    set(link12,'XData',[x_right_foot(j),x_right_foot(j)],'YData',[y_right_foot(j),y_right_foot(j)],'ZData',[z_right_foot(j),z_right_foot(j)]); 
    set(link13,'XData',[x_left_knee(j),x_left_knee(j)],'YData',[y_left_knee(j),y_left_knee(j)],'ZData',[z_left_knee(j),z_left_knee(j)]); 
    set(link14,'XData',[x_right_knee(j),x_right_knee(j)],'YData',[y_right_knee(j),y_right_knee(j)],'ZData',[z_right_knee(j),z_right_knee(j)]); 
    set(h11,'xdata',pcom(j,1),'ydata',pcom(j,2),'zdata',pcom(j,3));
    
%     view(3)
    az = 56;%90;%26;
    el = 17;
    
    view(az, el);
    axis equal
    xlim([-1 + j * 0.0073,5 + j * 0.0073])
    ylim([-1.2,1.2])
    zlim([-1.5,3])
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    set(gca,'ztick',[]);
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     zlabel('Z [m]');
%         axis('Position', [0, 0, 15, 10]);

%     set(gcf,'PaperPosition',[0, 10, 5, 15]);
    set(gca,'fontsize',17);
%     h111 = subplot(2,2,1);
%     set(h111, 'position',[10 20 10 20]);
%     subplot('Position',[0.1, 0.6, 0.3, 0.3]);
    %%%%%%%%%%%%%%%%%%%subplot2%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,2,2);
    set(link201mesh,'XData',xx_torso,'YData',yy_torso,'ZData',zz_torso, 'Facecolor', 'flat');
    rotate(link201mesh,[torso_rotate(1:3)],torso_rotate(4),torso_ellip_center);
    hold on;
    set(link20mesh,'XData',xx_hip,'YData',yy_hip,'ZData',zz_hip, 'Facecolor', 'flat');
    hold on;
    set(link22mesh,'XData',xx_leftthigh,'YData',yy_leftthigh,'ZData',zz_leftthigh, 'Facecolor', 'flat');
    rotate(link22mesh,[leftthigh_rotate(1:3)],leftthigh_rotate(4),leftthigh_ellip_center);
    hold on; 
    set(link21mesh,'XData',xx_leftcalf,'YData',yy_leftcalf,'ZData',zz_leftcalf, 'Facecolor', 'flat');
    rotate(link21mesh,[leftcalf_rotate(1:3)],leftcalf_rotate(4),leftcalf_ellip_center);
    hold on;
    set(link23mesh,'XData',xx_rightthigh,'YData',yy_rightthigh,'ZData',zz_rightthigh, 'Facecolor', 'flat');
    rotate(link23mesh,[rightthigh_rotate(1:3)],rightthigh_rotate(4),rightthigh_ellip_center);
    hold on; 
    set(link24mesh,'XData',xx_rightcalf,'YData',yy_rightcalf,'ZData',zz_rightcalf, 'Facecolor', 'flat');
    rotate(link24mesh,[rightcalf_rotate(1:3)],rightcalf_rotate(4),rightcalf_ellip_center);
    hold on; 
    
    patch([pcom(j-skipp,1) pcom(j,1)],[pcom(j-skipp,2) pcom(j,2)],[pcom(j-skipp,3) pcom(j,3)],'c');
    set(link201,'XData',[pcom(j,1) pcom(j,1) + 0.03],'YData',[pcom(j,2) pcom(j,2)],'ZData',[pcom(j,3) pcom(j,3) + L_torso]);%ptorso_tip is defined above
    set(link20,'XData',[x_left_hip(j),x_right_hip(j)],'YData',[y_left_hip(j),y_right_hip(j)],'ZData',[z_left_hip(j),z_right_hip(j)]);
    set(link21,'XData',[x_left_foot(j),x_left_foot(j)],'YData',[y_left_foot(j),y_left_foot(j)],'ZData',[z_left_foot(j),z_left_foot(j)]);
    set(link22,'XData',[x_right_foot(j),x_right_foot(j)],'YData',[y_right_foot(j),y_right_foot(j)],'ZData',[z_right_foot(j),z_right_foot(j)]);
    set(link23,'XData',[x_left_knee(j),x_left_knee(j)],'YData',[y_left_knee(j),y_left_knee(j)],'ZData',[z_left_knee(j),z_left_knee(j)]);
    set(link24,'XData',[x_right_knee(j),x_right_knee(j)],'YData',[y_right_knee(j),y_right_knee(j)],'ZData',[z_right_knee(j),z_right_knee(j)]);
    set(h21, 'xdata',pcom(j,1),'ydata',pcom(j,2),'zdata',pcom(j,3));
    
    view(3)
    %%%looking from the side view
    az = 0;
    el = 0;
    view(az, el);
    axis equal
    xlim([-1 + j * 0.0073,5 + j * 0.0073])
    ylim([-1.2,1.2])
    zlim([-1.5,3])
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    set(gca,'ztick',[]);
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     zlabel('Z [m]');
    set(gca,'fontsize',mediumTextSize);
    
    %%%%%%%%%%%%%%%%%%%subplot3%%%%%%%%%%%%%%%%%%%%%%%%
    subplot(2,2,4);
    set(link301mesh,'XData',xx_torso,'YData',yy_torso,'ZData',zz_torso, 'Facecolor', 'flat');
    rotate(link301mesh,[torso_rotate(1:3)],torso_rotate(4),torso_ellip_center);
    hold on; 
    set(link30mesh,'XData',xx_hip,'YData',yy_hip,'ZData',zz_hip, 'Facecolor', 'flat');
    hold on; 
    set(link33mesh,'XData',xx_leftthigh,'YData',yy_leftthigh,'ZData',zz_leftthigh, 'Facecolor', 'flat');
    rotate(link33mesh,[leftthigh_rotate(1:3)],leftthigh_rotate(4),leftthigh_ellip_center);
    hold on; 
    set(link31mesh,'XData',xx_leftcalf,'YData',yy_leftcalf,'ZData',zz_leftcalf, 'Facecolor', 'flat');
    rotate(link31mesh,[leftcalf_rotate(1:3)],leftcalf_rotate(4),leftcalf_ellip_center);
    hold on; 
    set(link32mesh,'XData',xx_rightcalf,'YData',yy_rightcalf,'ZData',zz_rightcalf, 'Facecolor', 'flat');
    rotate(link32mesh,[rightcalf_rotate(1:3)],rightcalf_rotate(4),rightcalf_ellip_center);
    hold on; 
    set(link34mesh,'XData',xx_rightthigh,'YData',yy_rightthigh,'ZData',zz_rightthigh, 'Facecolor', 'flat');
    rotate(link34mesh,[rightthigh_rotate(1:3)],rightthigh_rotate(4),rightthigh_ellip_center);
    hold on; 
    
    patch([pcom(j-skipp,1) pcom(j,1)],[pcom(j-skipp,2) pcom(j,2)],[pcom(j-skipp,3) pcom(j,3)],'c');
    set(link301,'XData',[pcom(j,1) pcom(j,1) + 0.03],'YData',[pcom(j,2) pcom(j,2)],'ZData',[pcom(j,3) pcom(j,3) + L_torso]);%ptorso_tip is defined above
    set(link30,'XData',[x_left_hip(j),x_right_hip(j)],'YData',[y_left_hip(j),y_right_hip(j)],'ZData',[z_left_hip(j),z_right_hip(j)]);
    set(link31,'XData',[x_left_foot(j),x_left_foot(j)],'YData',[y_left_foot(j),y_left_foot(j)],'ZData',[z_left_foot(j),z_left_foot(j)]); 
    set(link32,'XData',[x_right_foot(j),x_right_foot(j)],'YData',[y_right_foot(j),y_right_foot(j)],'ZData',[z_right_foot(j),z_right_foot(j)]); 
    set(link33,'XData',[x_left_knee(j),x_left_knee(j)],'YData',[y_left_knee(j),y_left_knee(j)],'ZData',[z_left_knee(j),z_left_knee(j)]); 
    set(link34,'XData',[x_right_knee(j),x_right_knee(j)],'YData',[y_right_knee(j),y_right_knee(j)],'ZData',[z_right_knee(j),z_right_knee(j)]); 
    set(h31, 'xdata',pcom(j,1),'ydata',pcom(j,2),'zdata',pcom(j,3));

    view(3)
    %looking downward
    az = 0;
    el = 90;
    view(az, el);
    axis equal
    xlim([-1 + j * 0.0073,5 + j * 0.0073])
    ylim([-1.2,1.2])
    zlim([-1.5,3])
    set(gca,'xtick',[]);
    set(gca,'ytick',[]);
    set(gca,'ztick',[]);
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     zlabel('Z [m]');
    set(gca,'fontsize',mediumTextSize);

    pause(0.001);
    drawnow
%     figureIndex = num2str(ii);%initialState
%     figurePath = ['figures/figure' figureIndex '.eps'];
%     
%     set(gcf, 'PaperPositionMode', 'auto');
%     print(figurePath,'-depsc','-r100');
    ii = ii + 1;        
    % MakeQTMovie('addframe');
end
% MakeQTMovie('finish');
% Finish by the Movie commend.