function runAtlasPDcontrol

options.floating = true;
m = RigidBodyModel('../models/mit_gazebo_models/mit_robot_drake/mit_drc_robot_minimal_contact.sdf',options);
    
dt = 0.001;
r = TimeSteppingRigidBodyManipulator(m,dt);

pgain = Point(r.getStateFrame);
pgain.neck_ay = 10;
pgain.l_leg_uay = 200;
pgain.l_leg_lax = 200;
pgain.r_leg_uay = 200;
pgain.r_leg_lax = 200;
pgain = double(pgain);

pgain(pgain==0) = 100;

nq = r.manip.getNumStates()/2;
kp = diag(pgain(1:nq)); 
kd = diag(2.0*ones(nq,1));

B = r.manip.model.B;
pos_mat = -B'*kp;
vel_mat = -B'*kd;

pd = LinearSystem([],[],[],[],[],[pos_mat,vel_mat]);

xstar = Point(r.getStateFrame);
xstar = r.manip.resolveConstraints(double(xstar));

% align frames so the goal is at the origin
if all(xstar==0)
  pd = setInputFrame(pd,r.getStateFrame);
else
  pd = setInputFrame(pd,CoordinateFrame([r.getStateFrame.name,' - ', mat2str(xstar,3)],length(xstar),r.getStateFrame.prefix));
  r.getStateFrame.addTransform(AffineTransform(r.getStateFrame,pd.getInputFrame,eye(length(xstar)),-xstar));
  pd.getInputFrame.addTransform(AffineTransform(pd.getInputFrame,r.getStateFrame,eye(length(xstar)),+xstar));
end
pd = setOutputFrame(pd,r.getInputFrame);

runDRCControl(r,pd,'mit_drc_robot','TRUE_ROBOT_STATE','ACTUATOR_CMDS',struct());

end

