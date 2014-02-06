function atlasVisualizer
%NOTEST

% a visualizer process for visualizing atlas state and relevant variables

% load robot model
r = Atlas();
load(strcat(getenv('DRC_PATH'),'/control/matlab/data/atlas_fp.mat'));
r = removeCollisionGroupsExcept(r,{'toe','heel'});
r = compile(r);
r = r.setInitialState(xstar);

% setup frames
state_plus_effort_frame = AtlasStateAndEffort(r);
state_plus_effort_frame.subscribe('EST_ROBOT_STATE');

nq = getNumDOF(r);
rfoot_ind = r.findLinkInd('r_foot');
lfoot_ind = r.findLinkInd('l_foot');

v = r.constructVisualizer;
lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'atlas-visualizer');

while true
  [x,t] = getNextMessage(state_plus_effort_frame,1);
  if ~isempty(x)
    tau = x(2*nq+(1:nq));
    q = x(1:nq);
    qd = x(nq+(1:nq));
    
    kinsol = doKinematics(r,q,false,true);
    com = getCOM(r,kinsol);
    cpos = contactPositions(r,q, [rfoot_ind, lfoot_ind]);

    % TODO: add ZMP via kalman filter estimate of qdd
    lcmgl.glColor3f(0, 0, 0);
    lcmgl.sphere([com(1:2)', min(cpos(3,:))], 0.015, 20, 20);

    lcmgl.switchBuffers();
    v.draw(t,x);
  end
end

end