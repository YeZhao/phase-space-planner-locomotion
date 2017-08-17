function [t_intersect_x,x_intersect_x,v_intersect_x] = find_speed_match_xz_func(vector1, vector2, t1, t2, inc_x, inc_t)
%find_speed_match_func(xinc', xinc2', t', t2', inc_x, inc_t);
pol_degree = 5;%6;%4; % the function polyfit is very sensitive to the degree. The higher the degree the harder is to find a polyfit that is well conditioned

% interpoloate sixth order polynomial to pcom and vcom forward trajectories
coeff_pol  = polyfit(t1,vector1, pol_degree);
coeff_pol_der  = polyder(coeff_pol);

% interpololate sixth order polynomial to pcom and vcom backward trajectories
coeff2_pol = polyfit(t2,vector2, pol_degree);
coeff2_pol_der = polyder(coeff2_pol);

% interpolate trajectories of inverse position-time forward / backward pcom trajecotries
[coeff_pol_t,s_pol_t,mu_pol_t] = polyfit(vector1,t1, pol_degree);
%coeff_pol_t   = polyfit(vector1,t1, pol_degree);
[coeff2_pol_t,s2_pol_t,mu2_pol_t] = polyfit(vector2,t2, pol_degree);
%coeff2_pol_t  = polyfit(vector2,t2, pol_degree);

% interpolate curve relating position-velocity forward trajectory
x_pol  = polyval(coeff_pol ,t1);
v_pol  = polyval(coeff_pol_der ,t1);
coeff_pos_vel = polyfit(x_pol,v_pol, pol_degree);
%[coeff_pos_vel,s_pos_vel,mu_pos_vel] = polyfit(x_pol,v_pol, pol_degree);

% interpolate curve relating position-velocity backward trajectory
x_pol2 = polyval(coeff2_pol,t2);
v_pol2 = polyval(coeff2_pol_der,t2);
coeff2_pos_vel  = polyfit(x_pol2,v_pol2, pol_degree);
%[coeff2_pos_vel,s2_pos_vel,mu2_pos_vel] = polyfit(x_pol2,v_pol2, pol_degree);

% find root of crossing position-velocity forward/backward trajectory
% this is the point where pcom and vcom are equal and therefore is the
% desire time crossing point
% this strategy is a contribution for the paper 

raices = roots(coeff_pos_vel-coeff2_pos_vel);

reales = raices(imag(raices)==0); % discard nor real solutions
rr = reales(reales>min(vector1)); % get point of intersect within range, part 1
x_intersect_x   = rr(rr<max(vector1)); % get point of intersect within rage, part 2
v_intersect_x   = polyval(coeff2_pos_vel, x_intersect_x); % get also velocity of intersect
%t1_intersect  = polyval(coeff_pol_t, x_intersect); % finally get time of intersect on forward trajectory
t1_intersect_x  = polyval(coeff_pol_t,x_intersect_x,[],mu_pol_t); % finally get time of intersect on forward trajectory
%t2_intersect  = polyval(coeff2_pol_t ,x_intersect); % finally get time of intersect on backward trajectory
t2_intersect_x  = polyval(coeff2_pol_t,x_intersect_x,[],mu2_pol_t); % finally get time of intersect on forward trajectory
t_intersect_x =[t1_intersect_x t2_intersect_x];

% plotting
% figure(8)
figure(2)
    clf
    plot(t1,x_pol,'b-.',t2,x_pol2,'r-.')
    grid on
    hold on
    xlabel('t [s]');
    ylabel('x [m]');
    drawnow

% figure(9)
figure(3)
    clf
    plot(x_pol,v_pol,'b-',x_pol2,v_pol2,'r-',x_intersect_x,v_intersect_x,'ko')
    xlabel('x [m]')
    ylabel('v [m/s]')
    drawnow




