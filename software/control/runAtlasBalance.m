function runAtlasBalance
options.floating = true;
options.view = 'right';
m = PlanarRigidBodyModel('../../ros_workspace/atlas_description/urdf/atlas_robot.urdf',options);
r = TimeSteppingRigidBodyManipulator(m,.005);
v = r.constructVisualizer;
v.display_dt = .05;

x0 = Point(r.getStateFrame);
u0 = Point(r.getInputFrame);
[xD,uD,success] = r.manip.resolveConstraintsFixedPoint(double(x0),double(u0));
num_q = r.getNumStates/2;
num_u = r.getNumInputs;
k_D = [-10*[zeros(num_u,num_q-num_u) eye(num_u)] -10*[zeros(num_u,num_q-num_u) eye(num_u)]];
k_y0 = -k_D*xD+uD;
control = AffineSystem([],[],[],[],[],[],[],k_D,k_y0);
control = control.setInputFrame(r.getOutputFrame);
control = control.setOutputFrame(r.getInputFrame);
r_cl = feedback(r,control);
%r_cl = cascade(feedback(r,control),v);
%simulate(r_cl,[0 1],x0);
xtraj = simulate(r_cl,[0 2],xD);
save PDcontrol.mat v xtraj;
v.playback(xtraj);
keyboard;
end

