%%%%%%%%%%
% USAGE  %
%%%%%%%%%%
%
% [tleft,yleft,dyleft,ddyleft...
% tright,yright,dyright,ddyright] =
% feet_forward_traj(...
%     footName, length, tFinish_previous, tStart, tFinish, dt_trajectory,...
%     tleft_traj_prev, yleft_traj_prev, dyleft_traj_prev, ddyleft_traj_prev,...
%     tright_traj_prev, yright_traj_prev, dyright_traj_prev, ddyright_traj_prev)
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
% y = position trajectories
% dy = velocity trajectories
% ddy = acceleration trajectories
% 
%%%%%%%%%%%%
% COMMENTS %
%%%%%%%%%%%%
%
% yStart is calculated automatically from previus trajectory data.
%

function [tleft,yleft,dyleft,ddyleft,tright,yright,dyright,ddyright] = feet_lateral_traj_func(...
    footName,y_init,length,tFinish_previous,tStart,tFinish,dt_trajectory,...
    tleft_traj_prev,yleft_traj_prev,dyleft_traj_prev,ddyleft_traj_prev,...
    tright_traj_prev,yright_traj_prev,dyright_traj_prev,ddyright_traj_prev)

% initializations
tleft = [];
yleft = [];
dyleft = [];
ddyleft = [];
tright = [];
yright = [];
dyright = [];
ddyright = [];

% initialize variables
tleft = tleft_traj_prev;
yleft = yleft_traj_prev;
dyleft = dyleft_traj_prev;
ddyleft = ddyleft_traj_prev;
tright = tright_traj_prev;
yright = yright_traj_prev;
dyright = dyright_traj_prev;
ddyright = ddyright_traj_prev;

% get start position of active foot
% yStart = y_init;
% if strcmp(footName,'left') & size(yleft,1) ~= 0
%     yStart = yleft(end);
% elseif strcmp(footName,'right') & size(yright,1) ~= 0
%     yStart = yright(end);
% end

% compute trajectory for the active foot
%[t,y,dy,ddy] = foot_forward_traj_func(yStart, length, tStart, tFinish, dt_trajectory);
[t,y,dy,ddy] = foot_lateral_traj_func(0, length, tStart, tFinish, dt_trajectory);

% get continuation times
if size(yleft,1) == 0
    yleft_cont = y_init;
    if tStart == 0
    yleft_cont = -y_init;
    end
else
    yleft_cont = yleft(end);

end
if size(yright,1) == 0
    yright_cont = y_init;
else
    yright_cont = yright(end);
end

% compute remaining trajectories of both feet
if strcmp(footName,'left') 
    tleft = [tleft_traj_prev, t];
    yleft = [yleft_traj_prev, yleft_cont + y];
    dyleft = [dyleft_traj_prev, dy];
    ddyleft = [ddyleft_traj_prev, ddy];
    tright = [tright_traj_prev, t];    
    yright = [yright_traj_prev, yright_cont * ones(1,size(t,2))];
    dyright = [dyright_traj_prev, zeros(1,size(t,2))];
    ddyright = [ddyright_traj_prev, zeros(1,size(t,2))];
elseif strcmp(footName,'right')
    tright = [tright_traj_prev, t];
    yright = [yright_traj_prev, yright_cont + y];
    dyright = [dyright_traj_prev, dy];
    ddyright = [ddyright_traj_prev, ddy];
    tleft = [tleft_traj_prev, t];
    yleft = [yleft_traj_prev, yleft_cont * ones(1,size(t,2))];
    dyleft = [dyleft_traj_prev, zeros(1,size(t,2))];
    ddyleft = [ddyleft_traj_prev, zeros(1,size(t,2))];
end