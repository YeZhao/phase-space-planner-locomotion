function [tcom, tcom_intersect, pcom, vcom, acom] = one_step_integration_single(p1, p2, pcom_i, pcom_f,...
    vcomx_i, vcomx_f, acomx_i, acomx_f, vcomy_i, acomy_i, pcom_intersect, inc_x, inc_t)
g = 9.81;

% decipher values
x1i = pcom_i(1);
y1i = pcom_i(2);
z1i = pcom_i(3);
x2f = pcom_f(1);
z2f = pcom_f(3);
p_intersect_x = pcom_intersect(1);
p_intersect_z = pcom_intersect(3);

% find dual line with intersections in sagittal plane
[pcom_traj1x, pcom_traj1z_x, a1_xz, b1_xz, pcom_traj2x, pcom_traj2z_x, a2_xz, b2_xz] = ...
create_dual_line_xz_func( x1i, z1i, x2f, z2f, p_intersect_x, p_intersect_z, inc_x );

figure(1)
clf
plot(pcom_traj1x,pcom_traj1z_x);
hold on;
plot(pcom_traj2x,pcom_traj2z_x,'r');
title('dual line from source');
grid on;

% Compute the xz sagittal dynamics
xinc  = pcom_traj1x(1);
vxinc  = vcomx_i;
axinc  = acomx_i;

% final condition [x, v, a]
xinc2 = pcom_traj2x(end);
vxinc2 = vcomx_f;
axinc2 = acomx_f; 

% estimation of sagittal plane trajectory
% estimation of forward trajectory by linear approximation
tx = 0; i = 2;
xinc = pcom_traj1x(1);  
zinc_x = a1_xz * xinc + b1_xz;

while xinc(end) < 1 * p2(1)   
    if xinc(end) < p_intersect_x
        zinc_x(i) = a1_xz * xinc(i-1) + b1_xz;
        axinc(i) = g * (xinc(i-1) - p1(1)) / ( ( zinc_x(i) - p1(3) ) - a1_xz * (xinc(i-1) - p1(1)) );
        vxinc(i) = vxinc(i-1) + axinc(i) * inc_t;
        xinc(i) = xinc(i-1) + vxinc(i) * inc_t + 0.5 * axinc(i) * inc_t^2;
        zinc_x(i) = a1_xz * xinc(i) + b1_xz;
        vzinc_x(i) = a1_xz * vxinc(i);
        azinc_x(i) = a1_xz * axinc(i);
        
    else 
        zinc_x(i) = a2_xz * xinc(i-1) + b2_xz;
        axinc(i) = g * (xinc(i-1) - p1(1)) / ( ( zinc_x(i) - p1(3) ) - a2_xz * (xinc(i-1) - p1(1)));
        vxinc(i) = vxinc(i-1) + axinc(i) * inc_t;
        xinc(i) = xinc(i-1) + vxinc(i) * inc_t + 0.5 * axinc(i) * inc_t^2;
        zinc_x(i) = a2_xz * xinc(i) + b2_xz;
        vzinc_x(i) = a2_xz * vxinc(i);
        azinc_x(i) = a2_xz * axinc(i);
    end
    tx(i) = tx(i-1) + inc_t;
    i = i + 1;
end

% estimation of backward trajectory by linear approximation
inc_t2 = -inc_t; % backwards recursion
tx2 = 0;  i = 2;
xinc2 = pcom_traj2x(end);  
zinc2_x = a2_xz * xinc2 + b2_xz;
while xinc2(end) > 1 * p1(1) % backwards
    if xinc2(end) > p_intersect_x
        zinc2_x(i) = a2_xz * xinc2(i-1) + b2_xz;
        axinc2(i) = g * (xinc2(i-1) - p2(1)) / ( ( zinc2_x(i) - p2(3) ) - a2_xz * (xinc2(i-1) - p2(1)));
        vxinc2(i) = vxinc2(i-1) + axinc2(i) * inc_t2;
        xinc2(i) = xinc2(i-1) + vxinc2(i) * inc_t2 + 0.5 * axinc2(i) * inc_t2^2;
        zinc2_x(i) = a2_xz * xinc2(i) + b2_xz;
        vzinc2_x(i) = a2_xz * vxinc2(i);  
        azinc2_x(i) = a2_xz * axinc2(i);
        
    else
        zinc2_x(i) = a1_xz * xinc2(i-1) + b1_xz;
        axinc2(i) = g * (xinc2(i-1) - p2(1)) / ( ( zinc2_x(i) - p2(3) ) - a1_xz * (xinc2(i-1) - p2(1)) );
        vxinc2(i) = vxinc2(i-1) + axinc2(i) * inc_t2;
        xinc2(i) = xinc2(i-1) + vxinc2(i) * inc_t2 + 0.5 * axinc2(i) * inc_t2^2;
        zinc2_x(i) = a1_xz * xinc2(i) + b1_xz;
        vzinc2_x(i) = a1_xz * vxinc2(i);  
        azinc2_x(i) = a1_xz * axinc2(i);
       
    end
    tx2(i) = tx2(i-1) + inc_t2;
    i = i+1;
end

[tt_intersect_x,x_intersect_x,v_intersect_x] = find_speed_match_xz_func(xinc', xinc2', tx', tx2', inc_x, inc_t);

% compute time shift
tx_shift = tt_intersect_x(1) - tt_intersect_x(2);
tx2 = tx2 + tx_shift;

% find indexes for t_intersect_x and t_intersect_z
indx1 = find( tx >= tt_intersect_x(1) ); % ind1(1) contains the index of t until the intersection
indx2 = find( tx2 <= tt_intersect_x(1) ); % ind2(1) contains the index of t2 from the intersection

% Compute the lateral dynamics
% estimation of forward trajectory by linear approximation
% initial condition [y, v, a]
yinc  = y1i;
vyinc  = vcomy_i;
ayinc  = acomy_i;

ty = 0; j = 2;
yinc = y1i;  
while j <= indx1(1)   
        ayinc(j) = g * (yinc(j-1) - p1(2)) / ( b1_xz + a1_xz * p1(1) - p1(3));
        vyinc(j) = vyinc(j-1) + ayinc(j) * inc_t;
        yinc(j) = yinc(j-1) + vyinc(j) * inc_t + 0.5 * ayinc(j) * inc_t^2;  
        ty(j) = ty(j-1) + inc_t;
        j = j + 1;
end

yinc1_end = yinc(end);
vyinc1_end = vyinc(end);

inc_t2 = inc_t; % backwards recursion
ty2 = 0;  j = 2;
yinc2 = yinc1_end;  
vyinc2 = vyinc1_end;
while j <= indx2(1)
        ayinc2(j) = g * (yinc2(j-1) - p2(2)) / ( b2_xz + a2_xz * p2(1) - p2(3));
        vyinc2(j) = vyinc2(j-1) + ayinc2(j) * inc_t2;
        yinc2(j) = yinc2(j-1) + vyinc2(j) * inc_t2 + 0.5 * ayinc2(j) * inc_t2^2;
        ty2(j) = ty2(j-1) + inc_t2;
        j = j+1;
end
yinc2_end = yinc2(end);

figure(4)
clf
plot(tx,xinc,'b-',tx2,xinc2,'r-')
grid on
hold on
xlabel('time [s]')
ylabel('x-pos [m]')
drawnow

figure(5)
clf
plot(ty,yinc,'b-',ty2,yinc2,'r-')
grid on
hold on
xlabel('time [s]')
ylabel('y-pos [m]')
drawnow

indy1 = indx1;
indy2 = indx2;

disp('checkcheckcheckcheckcheckcheckcheckcheck')
% flip reverse times
pcomx = xinc(1:indx1(1));
pcomx(end)
flipped_tmp = xinc2(1:indx2(1));
pcomx = [pcomx, fliplr(flipped_tmp)];

pcomy = yinc(1:indy1(1));
flipped_tmp = yinc2(1:indy2(1));
pcomy = [pcomy, flipped_tmp];

pcomz = zinc_x(1:indx1(1));
flipped_tmp = zinc2_x(1:indx2(1));
pcomz = [pcomz, fliplr(flipped_tmp)];

pcom = [pcomx; pcomy; pcomz];

vcomx = vxinc(1:indx1(1));
flipped_tmp = vxinc2(1:indx2(1));
vcomx = [vcomx, fliplr(flipped_tmp)];

vcomy = vyinc(1:indy1(1));
flipped_tmp = vyinc2(1:indy2(1));
vcomy = [vcomy, flipped_tmp];

vcomz = vzinc_x(1:indx1(1));
flipped_tmp = vzinc2_x(1:indx2(1));
vcomz = [vcomz, fliplr(flipped_tmp)];

vcom = [vcomx; vcomy; vcomz];

acomx = axinc(1:indx1(1));
flipped_tmp = axinc2(1:indx2(1));
acomx = [acomx, fliplr(flipped_tmp)];

acomy = ayinc(1:indy1(1));
flipped_tmp = ayinc2(1:indy2(1));
acomy = [acomy, flipped_tmp];

acomz = azinc_x(1:indx1(1));
flipped_tmp = azinc2_x(1:indx2(1));
acomz = [acomz, fliplr(flipped_tmp)];

acom = [acomx; acomy; acomz];

tcom_x = tx(1:indx1(1));
tcom_x_intersect = tcom_x(end);
flipped_tmp = tx2(1:indx2(1));
tcom_x = [tcom_x, fliplr(flipped_tmp)];

indy1 = indx1;
indy2 = indx2;

tcom = tcom_x;
tcom_intersect = tcom_x_intersect;
disp('useful');
