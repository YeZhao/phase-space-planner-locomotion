function [tleft,zleft,dzleft,ddzleft,tright,zright,dzright,ddzright] = ...
    feet_vertical_traj_func(footName, zStart, zEnd, Height, ...
    tFinish_previous, tStart, tFinish, vmax, dt_trajectory,...
    tleft, zleft, dzleft, ddzleft, tright, zright, dzright, ddzright)

duration = tFinish - tStart;
[t,z,dz,ddz] = foot_vertical_traj_func(zStart, zEnd, Height, tStart, ...
    duration, vmax, dt_trajectory);

% get continuation times
if size(zleft,1) == 0
    zleft_cont = zStart;
    %zleft_cont = zEnd;
else
    zleft_cont = zleft(end);
end
if size(zright,1) == 0
    zright_cont = zStart;
    %zright_cont = zEnd;
else
    zright_cont = zright(end);
end


if strcmp(footName,'left') 
    tleft = [tleft, t];
    zleft = [zleft, z];
    dzleft = [dzleft, dz];
    ddzleft = [ddzleft, ddz];
    tright = [tright, t];
    %zright = [zright, zeros(1,size(t,2))];
    zright = [zright, zright_cont * ones(1,size(t,2))];
    dzright = [dzright, zeros(1,size(t,2))];
    ddzright = [ddzright, zeros(1,size(t,2))];
elseif strcmp(footName,'right')
    tright = [tright, t];
    zright = [zright, z];
    dzright = [dzright, dz];
    ddzright = [ddzright, ddz];
    tleft = [tleft, t];
    %zleft = [zleft,zeros(1,size(t,2))];
    zleft = [zleft, zleft_cont * ones(1,size(t,2))];
    dzleft = [dzleft, zeros(1,size(t,2))];
    ddzleft = [ddzleft, zeros(1,size(t,2))];
end