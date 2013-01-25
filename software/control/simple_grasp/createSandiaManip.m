function [r] = createSandiaManip()
dt=0.001;
options.floating=false; % If this is set to true there are issues, need to fix them later
path=getenv('DRC');
filelocation=[path 'mit_robot_hands/drake_urdfs/model.sdf'];
r=TimeSteppingRigidBodyManipulator(filelocation,dt,options);
end