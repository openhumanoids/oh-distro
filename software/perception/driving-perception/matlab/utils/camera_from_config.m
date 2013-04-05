function cam = camera_from_config(cfg, key)

sub_cfg = cfg.get(cfg, key);

% image size
cam.width = sub_cfg.get(sub_cfg,'width');
cam.height = sub_cfg.get(sub_cfg,'height');

% extrinsic pose
cam.T = sub_cfg.get(sub_cfg,'position');
cam.R = rpy2rot(sub_cfg.get(sub_cfg,'rpy')/180*pi);

% pinhole matrix
pinhole = sub_cfg.get(sub_cfg,'pinhole');
cam.K = [pinhole(1),pinhole(3),pinhole(4);0,pinhole(2),pinhole(5);0,0,1];

% lens distortion
dist_model = sub_cfg.get(sub_cfg,'distortion_model');
if (strcmpi(dist_model,'spherical'))
    params = sub_cfg.get(sub_cfg,'distortion_params');
    lens = new_spherical_dist(params);
    lens.K = cam.K;
    cam.lens = lens;
end
