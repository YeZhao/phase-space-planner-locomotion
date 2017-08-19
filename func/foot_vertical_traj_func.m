%%%%%%%%%%
% USAGE  %
%%%%%%%%%%
%
% [t,z,dz,ddz] =
% foot_vertical_traj(z_start, zEnd, Height, tStart, TTotal, vmax, dt_trajectory)
%
%%%%%%%%%%%%%%%%
% INPUT VALUES %
%%%%%%%%%%%%%%%%
%
% zStart = start height
% zEnd = final height
% Height = desired height
% tStart = start time
% TTotal = total time to go up and down
% vmax = maximum velocity allowed (if TTotal is too small, then Height is automatically capped.
% dt_trajectory = increment of time between trajectory points. Recommended time is 0.0001 = 0.1ms.
%
%%%%%%%%%%%%%%%%%
% OUTPUT VALUES %
%%%%%%%%%%%%%%%%%
%
% t = time trajectory
% z = position trajectory
% dz = velocity trajectory
% ddz = acceleration trajectory

function [t, z, dz, ddz] = foot_vertical_traj_func(zStart, zEnd, Height, tStart, TTotal, vmax, dt_trajectory)

steadyOn = false;

% determine what segment is longer
Height1 = Height - zStart
Height2 = Height - zEnd

% reach max velocity on longer segment
% The kernel is zh = zStart + Height / 2 * ( 1 - cos( 2 * pi * th / ( 2 *
% THigh ) ) );
% the velocity max happens when th = Th / 2, then, dzh/dt = Height / 2 sin
% (2 * pi * Th/2) * pi / Th = Height * pi / (2 * Th)
if Height1 > Height2
   Th1 =  Height1 * pi / ( 2 * vmax );
   Th2 = Th1 * Height2 / Height1;
else
   Th2 =  Height2 * pi / ( 2 * vmax );
   Th1 = Th2 * Height1 / Height2;
end

% Dicrease height if can't go faster
if TTotal < ( Th1 + Th2 )
   Height1 = ( TTotal * Height1 / ( Height1 + Height2) ) * 2 * vmax / pi; 
   Height2 = zStart + Height1 - zEnd;
   Th1 = ( TTotal * Height1 / ( Height1 + Height2) );
   Th2 = ( TTotal * Height2 / ( Height1 + Height2) );
else
    steadyOn = true;
end

% Dicrease height if can't go faster
% if( THigh_min > TTotal / 2 )
%     Height = TTotal /2 * 2 * vmax / pi;
%     THigh = TTotal / 2;
% else
%     THigh = THigh_min;
% end

% raising phase
num = floor( Th1 / dt_trajectory );
th = linspace( 0, Th1, num );
zh = zStart + Height1 / 2 * ( 1 - cos( 2 * pi * th / ( 2 * Th1 ) ) );
dzh = Height1 / 2 * sin( 2 * pi * th / ( 2 * Th1 ) ) * pi / Th1;
ddzh = Height1 / 2 * cos( 2 * pi * th / ( 2 * Th1  ) ) * ( pi / Th1 )^2;

% steady phase
if( steadyOn )
    TSteady = TTotal - Th1 - Th2;
    num = floor( TSteady / dt_trajectory );
    ts = linspace( th(end), TTotal - Th2, num );
    zs = ones( 1, num ) * zh(end);
    dzs = zeros( 1, num);
    ddzs = zeros( 1, num);
end;

% descent phase

if( steadyOn )
    num = floor( Th2 / dt_trajectory );
    tl = linspace( ts(end), TTotal, num );
    TTotal
    num
    zl = zEnd + Height2 / 2 * ( 1 + cos( 2 * pi * ( tl - ts(end) ) / ( 2 * Th2 ) ) );
    dzl = - Height2 / 2 * sin( 2 * pi * ( tl - ts(end) ) / ( 2 * Th2 ) ) * pi / Th2;
    ddzl = - Height2 / 2 * cos( 2 * pi * ( tl - ts(end) ) / ( 2 * Th2  ) ) * ( pi / Th2 )^2;
    t = [th, ts, tl];
    z = [zh, zs, zl];
    dz = [dzh, dzs, dzl];
    ddz = [ddzh, ddzs, ddzl];
    disp('descent1');
else
    num = floor( Th2 / dt_trajectory );
    tl = linspace( th(end), TTotal, num )
    zl = zEnd + Height2 / 2 * ( 1 + cos( 2 * pi * ( tl - th(end) ) / ( 2 * Th2 ) ) );
    dzl = - Height2 / 2 * sin( 2 * pi * ( tl - th(end) ) / ( 2 * Th2 ) ) * pi / Th2;
    ddzl = - Height2 / 2 * cos( 2 * pi * ( tl - th(end) ) / ( 2 * Th2  ) ) * ( pi / Th2 )^2;
    t = [th, tl];
    z = [zh, zl];
    dz = [dzh, dzl];
    ddz = [ddzh, ddzl];
    disp('descent2');
end
t(end-1)
t(end)

tStart

t = t + tStart;
