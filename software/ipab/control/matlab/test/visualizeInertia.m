options.visualize = true;
options.floating = true;
options.terrain = RigidBodyFlatTerrain();

val_version = 3
if (val_version == 1)
  urdf = fullfile(getDrakePath(),'..','models','valkyrie','V1_sim_shells_reduced_polygon_count_mit.urdf');
  S = load([getDrakePath(), '/../control/matlab/data/valkyrie_fp.mat']);
elseif (val_version == 2)
  urdf = fullfile(getDrakePath(),'..','models','val_description','model.urdf');
  S = load([getDrakePath(), '/../control/matlab/data/valkyrie_fp_june2015.mat']);
else
  urdf = '/home/mfallon/otherprojects/val_description/robots/valkyrie_A.urdf';
  S = load([getDrakePath(), '/../control/matlab/data/valkyrie_fp_june2015.mat']);
end
  
r = RigidBodyManipulator(urdf,options);
r = r.compile();
v = r.constructVisualizer(); % robot arms spread, pelvise at zero
v = v.enableLCMGLInertiaEllipsoids();
nq = 38;
q_nom = S.xstar(1:nq);

%q_nom = S.xstar(1:65);
v.draw(0,q_nom); % standing configuration with left arm out to side