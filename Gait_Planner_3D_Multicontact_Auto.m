%====================================
% Author:       Ye Zhao
% Date:         September 4th. 2016
% File Name:     Gait_Planner_3D_Multicontact_Auto.m
%====================================


% This file is for testing the walking on the terrain with lateral slope.
% we should redefine all the boundary x1i and x2f each step after
% redefining the lateral location. This is possible.
clear all;

orange = [1,3/5,0];

[s,w] = system('dir');
if s % for macintosh and unix
    addpath([pwd,'/func'])
else % for windows
    addpath([pwd,'\func'])
end

%=================
% Constants
%=================

g = 9.81;
inc_x = 0.0005; % 0.5 [mm]
inc_t = 0.0001; % [s]

%=================
% initializations
%=================
tcom = [];

pcom = [];
vcom = [];
acom = [];

tcom_y = [];

pcom_y = [];
vcom_y = [];
acom_y = [];

tcom_intersect = [];
tcom_end = [];
ind_prev = 1;
feet_x_positions = [];
feet_y_positions = [];

% p11 = [-0.11; -0.04; 0.0];% first contact
% p21 = [0.45; 0.2580; 0.0];

y_intersect_random = 0;
y_random = 0;
p2_y_temp = 0;

disp('How many stairs you would like to walk on?(Input Integer Number)');
n = input('');
Hstair_rand = randsrc(1,n,[-1,1]) .* (0.20 * ones(1,n) + 0.1 * randsrc(1,n,[-1,1]) .* rand(1,n));

for i =1:length(Hstair_rand) + 2
    disp('Step'), disp(i);
    
    if i ==1
        p_base_height(i) = 0.0;
        p1(:,i) = [-0.11; -0.04; p_base_height(i)];%p1(:,i) first contact
        p2(:,i) = [0.30; y_random; p_base_height(i)]; %(:,i)
    else
        p1(:,i) = p2(:,i-1);% first contact
        
        if i< length(Hstair_rand) + 2
            if Hstair_rand(i-1) < 0 && p2(3,i-1) + Hstair_rand(i-1) < -1.2
                Hstair_rand(i-1) = - Hstair_rand(i-1);
            end
            if Hstair_rand(i-1) > 0 && p2(3,i-1) + Hstair_rand(i-1) > 1.2
                Hstair_rand(i-1) = - Hstair_rand(i-1);
            end
        end
        
        if i == length(Hstair_rand) + 1
            p_base_height(i) = p_base_height(i-1) + Hstair_rand(i-1);
            p2(:,i) = [p2(1,i-1) + 0.45; p2_y_temp; p_base_height(i)];
        elseif i == length(Hstair_rand) + 2
            p2(:,i) = [p2(1,i-1) + 0.2; p2_y_temp; p_base_height(i-1)];
        else
            p_base_height(i) = p_base_height(i-1) + Hstair_rand(i-1);
            p2(:,i) = [p2(1,i-1) + 0.6; p2_y_temp; p_base_height(i)];
        end
        % p1 = p2;% first contact
        % p2 = [p2(1) + 0.6; p2_y_temp; p2(3) + Hstair_rand(i-1)];
    end
    
    % com trajectory 1 - straight line trajectory
    if i == 1
        x1i = -0.06;
        y1i = 0;
        z1i = 1.08;
        
        % second part of trajectory
        x2f = p2(1,i);
        % x2f = p2(1);
        z2f = 1.1250;
    else
        
        x1i = pcom(1,end);
        y1i = pcom(2,end);
        z1i = pcom(3,end);
        % second part of trajectory
        if i == length(Hstair_rand) + 1
            x2f = p2(1,i);% - 0.1;
            z2f = z2f + Hstair_rand(i-1);
        elseif i == length(Hstair_rand) + 2
            x2f = p2(1,i);% - 0.1;
            z2f = z2f;
        else
            x2f = p2(1,i);
            z2f = z2f + Hstair_rand(i-1);
        end
    end
    
    % Intersection of trajectories
    % For pcom_intersect, we don't need lateral y value. So we assign the
    % second entry y_intersect_random randomly.
    
    pcom_intersect = [(x1i + x2f)/2.0, y_intersect_random, (z1i + z2f)/2-0.03];
    
    % other variables
    pcom_i = [x1i, y1i, z1i];
    % For the final vaule of pcom, we don't need y2f. y_random is just to make
    % pcom_f is also a 3X1 vector, consistent with pcom_i. y_random can be any value.
    pcom_f = [x2f, y_random, z2f];
    
    if i == 1
        vcomx_i = 0.10;
        acomx_i = 0.4805;
    else
        vcomx_i = vcom(1,end);
        acomx_i = acom(1,end);
    end
    if i == length(Hstair_rand) + 1
        vcomx_f = 0.3;
    elseif i == length(Hstair_rand) + 2
        vcomx_f = 0.05;
    else
        vcomx_f = 0.60;
    end
    acomx_f = 0.0;
    
    if i == 1
        vcomy_i = 0;
        acomy_i = 0;
    else
        vcomy_i = vcom(2,end);
        acomy_i = acom(2,end);
    end
    % Newton-Raphson Method to search the solutions
    % p21_initial = [0.4591; 0.4080; 0.0445];
    if mod(i,2) ==1
        p2_initial_y = 0.15;
        %     p2_initial(3) = p2(3,i) - 0.3 * p2_initial_y;
        
    else
        p2_initial_y = -0.15;
        %     p2_initial(3) = p2(3,i) + 0.3 * p2_initial_y;
    end
    
    
    p2_initial = [p2(1,i); p2_initial_y; p2(3,i)];
    % [p2(:,i), interation_steps] = NR_searching_method_multi(p1(:,i), p2_initial, pcom_i, pcom_f,...
    % vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t);
    
    [p2(:,i), pcom_f, pcom_intersect, interation_steps] = NR_searching_method_multi_test(p1(:,i), p2_initial, pcom_i, pcom_f,...
        vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t);
    
    [tcom_tmp, tcom_intersect_tmp, pcom_tmp, vcom_tmp, acom_tmp] = one_step_integration_multi(p1(:,i), p2(:,i), pcom_i, pcom_f,...
        vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t);
    
    if i == 1
        tcom = [tcom tcom_tmp];
        pcom = [pcom pcom_tmp];
        vcom = [vcom vcom_tmp];
        acom = [acom acom_tmp];
        tcom_intersect = [tcom_intersect tcom_intersect_tmp];
        tcom_end = [tcom_end tcom_tmp(end)];
    else
        tcom_one_step = tcom_tmp(end)
        tcom = [tcom tcom(end) + tcom_tmp];
        pcom = [pcom pcom_tmp];
        vcom = [vcom vcom_tmp];
        acom = [acom acom_tmp];
        tcom_intersect = [tcom_intersect tcom_intersect(end) + tcom_intersect_tmp];
        tcom_end = [tcom_end tcom_end(end) + tcom_tmp(end)];
    end
    
    ind_intersection = find(tcom > tcom_intersect_tmp);
    ind_intersection_num = ind_intersection(1) + size(tcom,2);
    
    feet_x_positions = [feet_x_positions; zeros(size(tcom_tmp,2),3)];
    feet_x_positions(:,1) = tcom;
    feet_x_positions(ind_prev : ind_prev + (ind_intersection(1)-1), 2) = p1(1,i);
    feet_x_positions(ind_prev + ind_intersection(1):end, 2) = p2(1,i);
    feet_x_positions(ind_prev : ind_prev + (ind_intersection(1)-1), 3) = p1(3,i);
    feet_x_positions(ind_prev + ind_intersection(1):end, 3) = p2(3,i);
    
    feet_y_positions = [feet_y_positions; zeros(size(tcom_tmp,2),3)];
    feet_y_positions(:,1) = tcom;
    feet_y_positions(ind_prev : ind_prev + (ind_intersection(1)-1), 2) = p1(2,i);
    feet_y_positions(ind_prev + ind_intersection(1):end, 2) = p2(2,i);
    feet_y_positions(ind_prev : ind_prev + (ind_intersection(1)-1), 3) = p1(3,i);
    feet_y_positions(ind_prev + ind_intersection(1):end, 3) = p2(3,i);
    
    ind_prev = size(feet_x_positions,1);
    
end

save HSTAIR_RAND.dat Hstair_rand -ASCII
foot_contact_x = [p1(1,1), p2(1,:)];
save FOOT_CONTACT_X.dat foot_contact_x -ASCII
%%%%%%%%%%%%%%%%%%%%%%%
['done!']
%%%%%%%%%%%%%%%%%%%%%%%

% Saving data as .mat file

pcom_struct_x = zeros(size(tcom,2),4);
pcom_struct(:,1) = tcom;
pcom_struct(:,2:4) = pcom';

vcom_struct = zeros(size(tcom,2),4);
vcom_struct(:,1) = tcom;
vcom_struct(:,2:4) = vcom';

acom_struct = zeros(size(tcom,2),4);
acom_struct(:,1) = tcom;
acom_struct(:,2:4) = acom';

feet_positions = [feet_x_positions(:,1) feet_x_positions(:,2) feet_y_positions(:,2) feet_x_positions(:,3)];
% pcom_struct = [pcom_struct(:,1) pcom_struct(:,2) pcom_struct(:,2) pcom_struct(:,3)];
% vcom_struct = [vcom_struct(:,1) vcom_struct(:,2) vcom_struct(:,2) vcom_struct(:,3)];
% acom_struct = [acom_struct(:,1) acom_struct(:,2) acom_struct(:,2) acom_struct(:,3)];

save FEET_AUTOMATIC_PLANE.dat feet_positions -ASCII
save COM_AUTOMATIC_PLANE.dat pcom_struct -ASCII
save VCOM_AUTOMATIC_PLANE.dat vcom_struct -ASCII
save ACOM_AUTOMATIC_PLANE.dat acom_struct -ASCII

%=================
%Plotting
%=================
bigTextSize = 20;
mediumTextSize = 18;
textSize = 24;
smallTextSize = 14;
legendMargin = 0.3;

% Kinematic Trajectories
% Sagittal Kinematic Path
figure(6)
clf
plot(pcom(1,:),pcom(3,:),'b-','color',orange,'linewidth',2.5,'MarkerSize',0.5);
hold on;
% plot([p11(1) p21(1)],[p11(3) p21(3)],'-','linewidth',1.5)
% plot(p11(1),p11(3),'b.','MarkerSize',20);
% plot(p21(1),p21(3),'b.','MarkerSize',20);
%
% plot([p12(1) p22(1)],[p12(3) p22(3)],'-','linewidth',1.5)
% plot(p12(1),p12(3),'b.','MarkerSize',20);
% plot(p22(1),p22(3),'b.','MarkerSize',20);
%
% plot([p13(1) p23(1)],[p13(3) p23(3)],'-','linewidth',1.5)
% plot(p13(1),p13(3),'b.','MarkerSize',20);
% plot(p23(1),p23(3),'b.','MarkerSize',20);

% plot([p14(1) p24(1)],[p14(3) p24(3)],'-','linewidth',1.5)
% plot(p14(1),p14(3),'b.','MarkerSize',20);
% plot(p24(1),p24(3),'b.','MarkerSize',20);
%
% plot([p15(1) p25(1)],[p15(3) p25(3)],'-','linewidth',1.5)
% plot(p15(1),p15(3),'b.','MarkerSize',20);
% plot(p25(1),p25(3),'b.','MarkerSize',20);
%
% plot([p16(1) p26(1)],[p16(3) p26(3)],'-','linewidth',1.5)
% plot(p16(1),p16(3),'b.','MarkerSize',20);
% plot(p26(1),p26(3),'b.','MarkerSize',20);
%
% plot([p17(1) p27(1)],[p17(3) p27(3)],'-','linewidth',1.5)
% plot(p17(1),p17(3),'b.','MarkerSize',20);
% plot(p27(1),p27(3),'b.','MarkerSize',20);
title('CoM Sagital Geometric Trajectory','fontsize',bigTextSize);
xlabel('x [m]','fontsize',mediumTextSize);
ylabel('z [m]','fontsize',mediumTextSize);
axis([-0.5,4,0,2.5]);
set(gca,'fontsize',mediumTextSize);
grid on
box on;
hold on;

set(gcf,'PaperPositionMode','auto');
set(gcf,'PaperPosition',[0, 0, 7, 5]);
zoom on;
h_legend = legend('CoM Trajectory','Foot Trajectory');
set(h_legend,'FontSize',12);
hold on;
print -djpeg CoM_Sagittal_Geometric_Trajectory

% Lateral Kinematic Path
figure(7)
clf
plot(pcom(1,:),pcom(2,:),'b-','color',orange,'linewidth',2.5,'MarkerSize',0.5);
hold on;
% plot([p11(1) p21(1)],[-p11(2) p21(2)],'-','linewidth',0.5)
% plot([p11(1) p13(1)],[p11(2) p13(2)],'-','linewidth',1.5)
%
% plot(p11(1),p11(2),'b.','MarkerSize',25);
% plot(p21(1),p21(2),'b.','MarkerSize',25);
% plot(p21(1),p21(2)-0.03,'b.','MarkerSize',10);
% plot(p21(1),p21(2)-0.06,'b.','MarkerSize',10);
% plot(p21(1),p21(2)-0.09,'b.','MarkerSize',10);
% plot(p21(1),p21(2)+0.03,'b.','MarkerSize',10);
% plot(p21(1),p21(2)+0.06,'b.','MarkerSize',10);
%
% plot(p12(1),p12(2),'b.','MarkerSize',25);
% plot(p22(1),p22(2)-0.03,'b.','MarkerSize',10);
% plot(p22(1),p22(2)-0.06,'b.','MarkerSize',10);
% plot(p22(1),p22(2)-0.09,'b.','MarkerSize',10);
% plot(p22(1),p22(2)+0.03,'b.','MarkerSize',10);
% plot(p22(1),p22(2)+0.06,'b.','MarkerSize',10);
%
% plot(p13(1),p13(2),'b.','MarkerSize',25);
% plot(p23(1),p23(2)-0.03,'b.','MarkerSize',10);
% plot(p23(1),p23(2)+0.03,'b.','MarkerSize',10);
% plot(p23(1),p23(2)+0.06,'b.','MarkerSize',10);
% plot(p23(1),p23(2)+0.09,'b.','MarkerSize',10);

% plot(p14(1),p14(2),'b.','MarkerSize',25);
% plot(p24(1),p24(2),'b.','MarkerSize',25);
% plot(p24(1),p24(2)-0.03,'b.','MarkerSize',10);
% plot(p24(1),p24(2)-0.06,'b.','MarkerSize',10);
% plot(p24(1),p24(2)-0.09,'b.','MarkerSize',10);
% plot(p24(1),p24(2)+0.03,'b.','MarkerSize',10);
%
% plot(p15(1),p15(2),'b.','MarkerSize',25);
% plot(p25(1),p25(2),'b.','MarkerSize',25);
% plot(p25(1),p25(2)-0.03,'b.','MarkerSize',10);
% plot(p25(1),p25(2)+0.03,'b.','MarkerSize',10);
% plot(p25(1),p25(2)+0.06,'b.','MarkerSize',10);
% plot(p25(1),p25(2)+0.09,'b.','MarkerSize',10);
%
% plot(p16(1),p16(2),'b.','MarkerSize',25);
% plot(p26(1),p26(2),'b.','MarkerSize',25);
% plot(p26(1),p26(2)-0.03,'b.','MarkerSize',10);
% plot(p26(1),p26(2)-0.06,'b.','MarkerSize',10);
% plot(p26(1),p26(2)-0.09,'b.','MarkerSize',10);
% plot(p26(1),p26(2)+0.03,'b.','MarkerSize',10);
% plot(p26(1),p26(2)+0.06,'b.','MarkerSize',10);
%
% plot(p17(1),p17(2),'b.','MarkerSize',25);
% plot(p27(1),p27(2),'b.','MarkerSize',25);
% plot(p27(1),p27(2)+0.03,'b.','MarkerSize',10);
% plot(p27(1),p27(2)+0.06,'b.','MarkerSize',10);
%
% plot([p11(1) p13(1)],[p11(2) p13(2)],'-','linewidth',1.5)
% plot([p13(1) p15(1)],[p13(2) p15(2)],'-','linewidth',1.5)
% plot([p15(1) p17(1)],[p15(2) p17(2)],'-','linewidth',1.5)
% plot([p17(1) p27(1)],[p17(2) -p27(2)],'-','linewidth',1.5)
%
% plot([p11(1) p12(1)],[-p11(2) p12(2)],'-','linewidth',1.5)
% plot([p12(1) p14(1)],[p12(2) p14(2)],'-','linewidth',1.5)
% plot([p14(1) p16(1)],[p14(2) p16(2)],'-','linewidth',1.5)
% plot([p16(1) p27(1)],[p16(2) p27(2)],'-','linewidth',1.5)
%
% plot(p11(1),-p11(2),'b.','MarkerSize',25);
% plot(p27(1),-p27(2),'b.','MarkerSize',25);

title('CoM Lateral Geometric Trajectory','fontsize',bigTextSize);
xlabel('x [m]','fontsize',mediumTextSize);
ylabel('y [m]','fontsize',mediumTextSize);
axis([-0.5 4.5 -0.3 0.4]);
set(gca,'fontsize',mediumTextSize);
h_legend = legend('CoM Trajectory','Foot Trajectory');
set(h_legend,'FontSize',12);
grid on
box on;
hold on;
print -djpeg CoM_Lateral_Geometric_Trajectory

% Sagittal State-Space Behaviors
figure(8);
hold on;
plot(pcom(1,:),vcom(1,:),'r-','linewidth',3.0)
axis([-0.5,3.5,0,1.4]);
set(gca,'fontsize',mediumTextSize);
title('CoM phase diagram x','fontsize',bigTextSize);
xlabel('x [m]','fontsize',mediumTextSize);
ylabel('vcom(x) [m/s]','fontsize',mediumTextSize);
grid on;
box on;
hold on;
set(gca,'fontsize',mediumTextSize);
print -djpeg CoM_Sagittal_Phase_Portrait
% h=line('color', [1 0 0], 'marker', '.', 'markersize', 40, 'erasemode', 'xor');
% n=length(pcom(1,:));
% i=2;
% while i<=n
%     set(h, 'xdata', pcom(1,i), 'ydata', vcom(1,i));
%     drawnow;
%     pause(0.001);
%     i= i+60;
% end

% Lateral State-Space Behaviors
figure(9)
plot(pcom(2,:),vcom(2,:),'b-','linewidth',3.0)
set(gca,'fontsize',mediumTextSize);
grid on;
title('CoM Lateral Phase Portrait','fontsize',bigTextSize);
xlabel('y [m]','fontsize',mediumTextSize);
ylabel('vcom(y) [m/s]','fontsize',mediumTextSize);
axis([-0.12 0.17 -.8 .5]);
hold on;
set(gca,'fontsize',mediumTextSize);
print -djpeg CoM_Lateral_Phase_Portrait_multicontact

% h=line('color', [1 0 0], 'marker', '.', 'markersize', 40, 'erasemode', 'xor');
% n=length(pcom(2,:));
% i=2;
% while i<=n
%     set(h, 'xdata', pcom(2,i), 'ydata', vcom(2,i));
%     drawnow;
%     pause(0.001);
%     i= i+60;
% end
%
% % CoM Sagittal Position in Time Domain
% figure(10)
% plot(tcom, pcom(1,:),'b-','linewidth',2.5);
% grid on;
% box on;
% hold on;
% title('CoM Sagittal Position VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('x [m]','fontsize',mediumTextSize);
% set(gca,'fontsize',mediumTextSize);
% grid on;
% hold on;
% print -djpeg CoM_Sagittal_Position_VS_time
%
% % CoM Lateral Position in Time Domain
% figure(11)
% plot(tcom, pcom(2,:),'b-','linewidth',2.5);
% grid on;
% box on;
% hold on;
% title('CoM Lateral Position VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('y [m]','fontsize',mediumTextSize);
% set(gca,'fontsize',mediumTextSize);
% grid on;
% print -djpeg CoM_Lateral_Position_VS_time
%
% % CoM Vertical Position in Time Domain
% figure(12)
% plot(tcom, pcom(3,:),'b-','linewidth',2.5);
% axis([0 6 0 1.5]);
% title('CoM Vertical Position VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('z [m]','fontsize',mediumTextSize);
% set(gca,'fontsize',mediumTextSize);
% grid on;
% grid on;
% box on;
% hold on;
% print -djpeg CoM_Vertical_Position_VS_time
%
% % CoM Sagittal Velocity in Time Domain
% figure(13)
% plot(tcom, vcom(1,:),'b-','linewidth',2.5);
% grid on;
% box on;
% title('CoM Sagittal Velocity VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('vcom(x) [m/s]','fontsize',mediumTextSize);
% hold on;
% set(gca,'fontsize',mediumTextSize);
% grid on;
% print -djpeg CoM_Sagittal_Velocity_VS_time
%
% % CoM_Lateral_Velocity in Time Domain
% figure(14)
% plot(tcom,vcom(2,:),'b-','linewidth',2.5);
% title('CoM Lateral Acceleration VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('vcom(y) [m/s^2]','fontsize',mediumTextSize);
% grid on;
% box on;
% hold on;
% set(gca,'fontsize',mediumTextSize);
% print -djpeg CoM_Lateral_Velocity_VS_time
%
% % CoM Vertical Velocity in Time Domain
% figure(15)
% plot(tcom, vcom(3,:),'b-','linewidth',2.5);
% grid on;
% box on;
% hold on;
% title('CoM Vertical Velocity VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('vcom(z) [m/s]','fontsize',mediumTextSize);
% set(gca,'fontsize',mediumTextSize);
% print -djpeg CoM_Vertical_Velocity_VS_time
%
% % CoM_Sagittal_Acceleration in Time Domain
% figure(16)
% plot(tcom,acom(1,:),'b-','linewidth',2.5);
% grid on;
% box on;
% hold on;
% set(gca,'fontsize',mediumTextSize);
% title('CoM Sagittal Acceleration VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('acom(x) [m/s^2]','fontsize',mediumTextSize);
% print -djpeg CoM_Sagittal_Acceleration_VS_time
%
% % CoM_Lateral_Acceleration in Time Domain
% figure(17)
% plot(tcom,acom(2,:),'b-','linewidth',2.5);
% grid on;
% box on;
% set(gca,'fontsize',mediumTextSize);
% title('CoM Lateral Acceleration VS Time','fontsize',bigTextSize);
% xlabel('t [s]','fontsize',mediumTextSize);
% ylabel('acom(y) [m/s^2]','fontsize',mediumTextSize);
% print -djpeg CoM_Lateral_Acceleration_VS_time
%
% % CoM Vertical Acceleration in Time Domain
% figure(18)
% plot(tcom, acom(3,:),'b-','linewidth',2.5);
% grid on;
% box on;
% hold on;
% title('CoM Vertical Acceleration VS Time','fontsize',bigTextSize);
% xlabel('t [m]','fontsize',mediumTextSize);
% ylabel('acom(z) [m/s^2]','fontsize',mediumTextSize);
% set(gca,'fontsize',mediumTextSize);
% print -djpeg CoM_Vertical_Acceleration_VS_time
%
% %CoM_Sagittal_Acceleration_VS_Position
% figure(19)
% plot(pcom(1,:),acom(1,:),'b-','linewidth',2.5);
% grid on;
% box on;
% set(gca,'fontsize',mediumTextSize);
% title('CoM Sagittal Acceleration VS Position','fontsize',bigTextSize);
% xlabel('x [m]','fontsize',mediumTextSize);
% ylabel('acom(x) [m/s^2]','fontsize',mediumTextSize);
% hold on;
% print -djpeg CoM_Sagittal_Acceleration_VS_Position
%
% % CoM Lateral Acceleration VS Position
% figure(20)
% plot(pcom(1,:),acom(2,:),'b-','linewidth',2.5)
% grid on;
% box on;
% title('CoM Lateral Acceleration VS Position','fontsize',bigTextSize);
% xlabel('x [m]','fontsize',mediumTextSize);
% ylabel('acom(y) [m/s^2]','fontsize',mediumTextSize);
% set(gca,'fontsize',mediumTextSize);
% hold on;
% print -djpeg CoM_Lateral_Acceleration_VS_position
