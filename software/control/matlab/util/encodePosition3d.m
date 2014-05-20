function msg = encodePosition3d(pose)
	msg = drc.position_3d_t();
  msg.translation = drc.vector_3d_t();
  msg.rotation = drc.quaternion_t();
  msg.translation.x = pose(1);
  msg.translation.y = pose(2);
  msg.translation.z = pose(3);
  q = rpy2quat(pose(4:6));
  msg.rotation.w = q(1);
  msg.rotation.x = q(2);
  msg.rotation.y = q(3);
  msg.rotation.z = q(4);
end
