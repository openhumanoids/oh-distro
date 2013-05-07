function runPrecomputeServer()

addpath(fullfile(getDrakePath,'examples','ZMP'));

% load atlas model
options.floating = true;
r = Atlas(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
r = removeCollisionGroupsExcept(r,{'heel','toe'});
r = compile(r);

ps = PrecomputeServer(r);
ps = addPrecomputeNode(ps,'STANDING_PREC_REQUEST',@precomputeNodeTIZMP);
ps.run();

end


