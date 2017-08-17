%%%%%%%%%%
% USAGE  %
%%%%%%%%%%
%
% [tleft,xleft,dxleft,ddxleft...
% tright,xright,dxright,ddxright] =
% feet_forward_traj(...
%     footName, length, tFinish_previous, tStart, tFinish, dt_trajectory,...
%     tleft_traj_prev, xleft_traj_prev, dxleft_traj_prev, ddxleft_traj_prev,...
%     tright_traj_prev, xright_traj_prev, dxright_traj_prev, ddxright_traj_prev)
%
%%%%%%%%%%%%%%%%
% INPUT VALUES %
%%%%%%%%%%%%%%%%
%
% footname = name of moving foot: 'left' or 'right'
% length = length stroke active foot
% tFinish_previous = finish of previous portion of movement for both feet
%   (this is normally the end of zmp transfer function)
% tStart = start time for both feet
% tFinish = final time for both feet
% dt_trajectory = increment of time between trajectory points. Recommended
%   time is 0.0001 = 0.1ms
% ..._traj_prev = all previous trajectories
%
%%%%%%%%%%%%%%%%%
% OUTPUT VALUES %
%%%%%%%%%%%%%%%%%
%
% t = time trajectory
% x = position trajectories
% dx = velocity trajectories
% ddx = acceleration trajectories
% 
%%%%%%%%%%%%
% COMMENTS %
%%%%%%%%%%%%
%
% xStart is calculated automatically from previus trajectory data.
%

function [tleft,xleft,dxleft,ddxleft,tright,xright,dxright,ddxright] = feet_forward_traj_func(...
    footName,x_init,length,tFinish_previous,tStart,tFinish,dt_trajectory,...
    tleft_traj_prev,xleft_traj_prev,dxleft_traj_prev,ddxleft_traj_prev,...
    tright_traj_prev,xright_traj_prev,dxright_traj_prev,ddxright_traj_prev)

% initializations
tleft = [];
xleft = [];
dxleft = [];
ddxleft = [];
tright = [];
xright = [];
dxright = [];
ddxright = [];

% initialize variables
tleft = tleft_traj_prev;
xleft = xleft_traj_prev;
dxleft = dxleft_traj_prev;
ddxleft = ddxleft_traj_prev;
tright = tright_traj_prev;
xright = xright_traj_prev;
dxright = dxright_traj_prev;
ddxright = ddxright_traj_prev;

% get start position of active foot
xStart = x_init;
% if strcmp(footName,'left') & size(xleft,1) ~= 0
%     xStart = xleft(end);
% elseif strcmp(footName,'right') & size(xright,1) ~= 0
%     xStart = xright(end);
% end

% compute trajectory for the active foot
%[t,x,dx,ddx] = foot_forward_traj_func(xStart, length, tStart, tFinish, dt_trajectory);
[t,x,dx,ddx] = foot_forward_traj_func(0, length, tStart, tFinish, dt_trajectory);

% get continuation times
if size(xleft,1) == 0
    xleft_cont = x_init;
else
    xleft_cont = xleft(end);
end
if size(xright,1) == 0
    xright_cont = x_init;
else
    xright_cont = xright(end);
end

% compute remaining trajectories of both feet
if strcmp(footName,'left') 
    tleft = [tleft_traj_prev, t];
    xleft = [xleft_traj_prev, xleft_cont + x];
    dxleft = [dxleft_traj_prev, dx];
    ddxleft = [ddxleft_traj_prev, ddx];
    tright = [tright_traj_prev, t];    
    xright = [xright_traj_prev, xright_cont * ones(1,size(t,2))];
    dxright = [dxright_traj_prev, zeros(1,size(t,2))];
    ddxright = [ddxright_traj_prev, zeros(1,size(t,2))];
elseif strcmp(footName,'right')
    tright = [tright_traj_prev, t];
    xright = [xright_traj_prev, xright_cont + x];
    dxright = [dxright_traj_prev, dx];
    ddxright = [ddxright_traj_prev, ddx];
    tleft = [tleft_traj_prev, t];
    xleft = [xleft_traj_prev, xleft_cont * ones(1,size(t,2))];
    dxleft = [dxleft_traj_prev, zeros(1,size(t,2))];
    ddxleft = [ddxleft_traj_prev, zeros(1,size(t,2))];
end