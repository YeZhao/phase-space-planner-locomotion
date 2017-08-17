function [tcom, tcom_intersect, pcom, vcom, acom] = one_step_integration_single_spec(p1, p2, pcom_i, pcom_f,...
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
[pcom_traj1x, pcom_traj1z_x, a1_xz, b1_xz] = ...
create_dual_line_xz_spec_func( x1i, z1i, x2f, z2f, inc_x );

figure(1)
clf
plot(pcom_traj1x,pcom_traj1z_x);
hold on;
grid on;

% Compute the xz sagittal dynamics
xinc  = pcom_traj1x(1);
vxinc  = vcomx_i;
axinc  = acomx_i;

% final condition [x, v, a]
xinc2 = pcom_traj1x(end);
vxinc2 = vcomx_f;
axinc2 = acomx_f; 

% estimation of sagittal plane trajectory
% estimation of forward trajectory by linear approximation
tx = 0; i = 2;
xinc = pcom_traj1x(1);  
zinc_x = a1_xz * xinc + b1_xz;


% lateral info
yinc  = y1i;
vyinc  = vcomy_i;
ayinc  = acomy_i;

ty = 0; j = 2;
yinc = y1i;  

while xinc(end) < 1 * p2(1) && yinc(end) >= 0 
        zinc_x(i) = a1_xz * xinc(i-1) + b1_xz;
        axinc(i) = g * (xinc(i-1) - p1(1)) / ( ( zinc_x(i) - p1(3) ) - a1_xz * (xinc(i-1) - p1(1)) );
        vxinc(i) = vxinc(i-1) + axinc(i) * inc_t;
        xinc(i) = xinc(i-1) + vxinc(i) * inc_t + 0.5 * axinc(i) * inc_t^2;
        zinc_x(i) = a1_xz * xinc(i) + b1_xz;
        vzinc_x(i) = a1_xz * vxinc(i);
        azinc_x(i) = a1_xz * axinc(i);
        
        j =i;
        ayinc(j) = g * (yinc(j-1) - p1(2)) / ( b1_xz + a1_xz * p1(1) - p1(3));
        vyinc(j) = vyinc(j-1) + ayinc(j) * inc_t;
        yinc(j) = yinc(j-1) + vyinc(j) * inc_t + 0.5 * ayinc(j) * inc_t^2;  
        ty(j) = ty(j-1) + inc_t;
        j = j + 1;
        
        
        tx(i) = tx(i-1) + inc_t;
        i = i + 1;
end
indx = size(xinc,2);


% 
% % Compute the lateral dynamics
% % estimation of forward trajectory by linear approximation
% % initial condition [y, v, a]
% 
% while j <= indx(1)   
%         
% end

yinc1_end = yinc(end);
vyinc1_end = vyinc(end);

figure(4)
clf
plot(tx,xinc,'b-')
grid on
hold on
xlabel('time [s]')
ylabel('x-pos [m]')
drawnow

figure(5)
clf
plot(ty,yinc,'b-')
grid on
hold on
xlabel('time [s]')
ylabel('y-pos [m]')
drawnow


disp('checkcheckcheckcheckcheckcheckcheckcheck')
% flip reverse times
pcomx = xinc(1:indx(1));


pcomy = yinc(1:indx(1));

pcomz = zinc_x(1:indx(1));


pcom = [pcomx; pcomy; pcomz];

vcomx = vxinc(1:indx(1));

vcomy = vyinc(1:indx(1));

vcomz = vzinc_x(1:indx(1));

vcom = [vcomx; vcomy; vcomz];

acomx = axinc(1:indx(1));

acomy = ayinc(1:indx(1));

acomz = azinc_x(1:indx(1));

acom = [acomx; acomy; acomz];

tcom_x = tx(1:indx(1));
tcom_intersect = tcom_x(end)/2;
tcom = tcom_x;
 
