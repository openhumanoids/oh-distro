function [r] = createSandiaManip()
dt=0.001;
options.floating=false; % If this is set to true there are issues, need to fix them later
drc_path=getenv('DRC_PATH');
filelocation=[drc_path 'models/mit_gazebo_models/mit_robot_hands/drake_urdfs/model.sdf'];
r=TimeSteppingRigidBodyManipulator(filelocation,dt,options);

%m = RigidBodyModel(filelocation,options);
%r = TimeSteppingRigidBodyManipulator(m,dt);
end
