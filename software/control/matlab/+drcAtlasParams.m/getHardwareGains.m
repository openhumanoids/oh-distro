function gains = getHardwareGains(r, force_controlled_joint_names)
typecheck(r, 'DRCAtlas');
typecheck(force_controlled_joint_names, 'cell');

force_controlled_joints = [];
for i=1:length(force_control_joint_names)
  force_controlled_joints = union(force_controlled_joints,find(~cellfun(@isempty,strfind(r.getInputFrame.coordinates,force_controlled_joint_names{i}))));
end
act_ind = (1:r.getNumInputs)';
position_controlled_joints = setdiff(act_ind,force_controlled_joints);

gains = getAtlasGains(r.atlas_version);
gains.k_q_p(force_controlled_joints) = 0;
gains.k_q_i(force_controlled_joints) = 0;
gains.k_qd_p(force_controlled_joints) = 0;
gains.k_f_p(position_controlled_joints) = 0;
gains.ff_f_d(position_controlled_joints) = 0;
gains.ff_qd(position_controlled_joints) = 0;
gains.ff_qd_d(position_controlled_joints) = 0;
