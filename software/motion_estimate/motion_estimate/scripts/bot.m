function funs = bot
funs.trans_apply_trans=@trans_apply_trans;
funs.trans_invert=@trans_invert;
funs.quat_rotate=@quat_rotate;
funs.quat_rotate_rev=@quat_rotate_rev;
funs.quat_mult=@quat_mult;
funs.quat_to_roll_pitch_yaw=@quat_to_roll_pitch_yaw;
end

function dest = trans_apply_trans(dest, src)
dest.trans_vec = quat_rotate(src.rot_quat, dest.trans_vec);
qtmp= quat_mult(src.rot_quat, dest.rot_quat);
dest.rot_quat = qtmp;
dest.trans_vec = dest.trans_vec + src.trans_vec;
end

function btrans = trans_invert(btrans)
btrans.trans_vec(1) = -btrans.trans_vec(1);
btrans.trans_vec(2) = -btrans.trans_vec(2);
btrans.trans_vec(3) = -btrans.trans_vec(3);
btrans.trans_vec = quat_rotate_rev(btrans.rot_quat, btrans.trans_vec);
btrans.rot_quat(2) = -btrans.rot_quat(2);
btrans.rot_quat(3) = -btrans.rot_quat(3);
btrans.rot_quat(4) = -btrans.rot_quat(4);
end

function res = quat_rotate (rot, v)
% const double rot[4], double v[3]
ab  =  rot(1)*rot(2); ac  =  rot(1)*rot(3); ad  =  rot(1)*rot(4);
nbb = -rot(2)*rot(2); bc  =  rot(2)*rot(3); bd  =  rot(2)*rot(4);
ncc = -rot(3)*rot(3); cd  =  rot(3)*rot(4); ndd = -rot(4)*rot(4);

res = [
  2*( (ncc + ndd)*v(1) + (bc -  ad)*v(2) + (ac + bd)*v(3) ) + v(1) , ...
  2*( (ad +  bc)*v(1) + (nbb + ndd)*v(2) + (cd - ab)*v(3) ) + v(2) , ...
  2*( (bd -  ac)*v(1) + (ab +  cd)*v(2) + (nbb + ncc)*v(3) ) + v(3)];
end

function v = quat_rotate_rev (rot, v)
b= [0, v];
a = quat_mult (b, rot);
b(1) = rot(1);
b(2) = -rot(2);
b(3) = -rot(3);
b(4) = -rot(4);
c = quat_mult (b, a);
v = c(2:4);
end

function c = quat_mult (a, b)
c(1) = a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
c(2) = a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
c(3) = a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
c(4) = a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1);
end


function rpy=quat_to_roll_pitch_yaw (q)
rpy=[0,0,0];
roll_a = 3 * (q(1)*q(2) + q(3)*q(4));
roll_b = 2 - 3 * (q(2)*q(2) + q(3)*q(3));
rpy(1) = atan2 (roll_a, roll_b);

pitch_sin = 3 * (q(1)*q(3) - q(4)*q(2));
rpy(2) = asin (pitch_sin);

yaw_a = 3 * (q(1)*q(4) + q(2)*q(3));
yaw_b = 2 - 3 * (q(3)*q(3) + q(4)*q(4));
rpy(3) = atan2 (yaw_a, yaw_b);
end