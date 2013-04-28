function out = synchronize_poses(in,t_out)

t_out = t_out(:);
pose_inds = closest_indices([in.t], t_out);
out_inds = find(pose_inds>0);
pose_inds = pose_inds(out_inds);
t_out = t_out(out_inds);
t1 = [in(pose_inds).t]';
t2 = [in(pose_inds+1).t]';
t_frac = (t_out-t1)./(t2-t1);
pose1 = in(pose_inds);
pose2 = in(pose_inds+1);
out = pose1;
for i = 1:numel(out)
    f = t_frac(i);
    out(i).t = t_out(i);
    out(i).position = (1-f)*pose1(i).position + f*pose2(i).position;
    out(i).orientation = (1-f)*pose1(i).orientation + f*pose2(i).orientation;
    out(i).orientation = out(i).orientation / norm(out(i).orientation);
    out(i).out_ind = out_inds(i);
end
