%%%%%%%%%%
% USAGE  %
%%%%%%%%%%
%
% [t,x,dx,ddx] =
% foot_forward_traj(xStart, length, tStart, tFinish, dt_trajectory)
%
%%%%%%%%%%%%%%%%
% INPUT VALUES %
%%%%%%%%%%%%%%%%
%
% xStart = start position
% length = length stroke
% tStart = start time
% TFinish = final time
% dt_trajectory = increment of time between trajectory points. Recommended time is 0.0001 = 0.1ms.
%
%%%%%%%%%%%%%%%%%
% OUTPUT VALUES %
%%%%%%%%%%%%%%%%%
%
% t = time trajectory
% x = position trajectory
% dx = velocity trajectory
% ddx = acceleration trajectory

function [t, x, dx, ddx] = foot_forward_traj_func(xStart, length, tStart, tFinish, dt_trajectory)

% raising phase
TTotal = tFinish - tStart;
num = round( TTotal / dt_trajectory );
t = linspace( 0, TTotal, num );
x = xStart + length / 2 * ( 1 - cos( 2 * pi * t / ( 2 * TTotal ) ) );
dx = length / 2 * sin( 2 * pi * t / ( 2 * TTotal ) ) * pi / TTotal;
ddx = length / 2 * cos( 2 * pi * t / ( 2 * TTotal  ) ) * ( pi / TTotal )^2;
t = t + tStart;


