function [p_traj_x1, p_traj_z1, aa1, bb1] = ...
create_dual_line_xz_spec_func( p_start_x1, p_start_z1, p_end_x2, p_end_z2, p_inc )

aa1 =  (p_end_z2 - p_start_z1)/(p_start_z1 - p_start_x1);
bb1 = (p_start_z1 * p_end_x2 - p_start_x1 * p_end_z2)/(p_end_x2 - p_start_x1);
p_traj_x1 = p_start_x1:p_inc:p_end_x2;
p_traj_z1 = aa1 * p_traj_x1 + bb1;

