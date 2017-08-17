function [p_traj_x1, p_traj_z1, aa1, bb1, p_traj_x2, p_traj_z2, aa2, bb2] = ...
create_dual_line_xz_func( p_start_x1, p_start_z1, p_end_x2, p_end_z2, p_intersect_x, p_intersect_z, p_inc )

aa1 =  (p_intersect_z - p_start_z1)/(p_intersect_x - p_start_x1);
bb1 = (p_start_z1 * p_intersect_x - p_start_x1 * p_intersect_z)/(p_intersect_x - p_start_x1);
p_traj_x1 = p_start_x1:p_inc:p_intersect_x;
p_traj_z1 = aa1 * p_traj_x1 + bb1;

aa2 =  (p_end_z2 - p_intersect_z)/(p_end_x2 - p_intersect_x);
bb2 = (p_intersect_z * p_end_x2 - p_intersect_x * p_end_z2)/(p_end_x2 - p_intersect_x);
p_traj_x2 = p_intersect_x:p_inc:p_end_x2;
p_traj_z2 = aa2 * p_traj_x2 + bb2;
