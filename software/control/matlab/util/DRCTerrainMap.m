classdef DRCTerrainMap < RigidBodyTerrain

  methods
    function obj = DRCTerrainMap(is_robot,options)

      if nargin < 1
        is_robot = false;
      end


      if is_robot
        private_channel = true;
      else
        private_channel = false;
      end

      if nargin < 2
        options = struct();
      else
        typecheck(options,'struct');
      end

      if isfield(options,'name');
        typecheck(options.name,'char');
      else
        options.name = '';
      end

      if isfield(options,'status_code')
        typecheck(options.status_code, 'numeric');
      else
        options.status_code = 3;
      end

      if isfield(options,'normal_radius')
        % Radius (in pixels) around each point to use for smoothing normals
        typecheck(options.normal_radius, 'numeric');
      else
        options.normal_radius = 2;
      end

      if isfield(options,'normal_method')
        typecheck(options.normal_method, 'char');
      else
        options.normal_method = 'ransac';
      end
      obj.default_normal_method = options.normal_method;

      if isfield(options,'raw')
        typecheck(options.raw, 'logical');
      else
        options.raw = false;
      end

      if isfield(options,'listen_for_foot_pose')
        % Whether to listen to the POSE_FOOT lcm channel to get the fill plane
        typecheck(options.listen_for_foot_pose, {'logical', 'numeric'});
      else
        options.listen_for_foot_pose = true;
      end

      if (private_channel)
          obj.map_handle = HeightMapHandle(@HeightMapWrapper,'true');
      else
          obj.map_handle = HeightMapHandle(@HeightMapWrapper,'false');
      end

      obj.map_handle.setFillMissing(~options.raw);
      obj.map_handle.setUseFootPose(options.listen_for_foot_pose);
      obj.map_handle.setNormalRadius(options.normal_radius);
      obj.map_handle.setNormalMethod(options.normal_method);

    end

    function [z,normal] = getHeight(obj,xy)
      [z,normal] = obj.map_handle.getTerrain(xy);
    end

    function obj = setFillPlane(obj,plane)
      % Set fill plane explicitly.
      % @param plane a 4-vector [v; b] such that v' * [x;y;z] + b == 0
      obj.map_handle.setUseFootPose(false);
      obj.map_handle.setFillPlane(plane)
    end

    function obj = setFillPlaneFromConfiguration(obj, biped, q, force_z_normal, frame_name)
      % Use a configuration of the robot to set the fill plane to
      % the plane at a specified frame on the robot (the right foot
      % sole by default)
      if nargin < 5
        frame_name = 'r_foot_sole';
      end
      sizecheck(q, [biped.getNumDOF, 1]);
      fid = biped.findFrameId(frame_name);
      kinsol = doKinematics(biped, q);
      fpos = forwardKin(biped, kinsol, fid, [0;0;0], true);
      if force_z_normal
        v = [0;0;1];
      else
        v = rpy2rotmat(fpos(4:6)) * [0;0;1];
      end
      b = -v' * fpos(1:3);

      obj = obj.setFillPlane([v;b]);
    end

    function obj = setMapMode(obj,mode)
      obj.map_handle.setMapMode(mode);
    end

    function obj = overrideNormals(obj, override)
      if override
        obj.map_handle.setNormalMethod('override');
      else
        obj.map_handle.setNormalMethod(obj.default_normal_method);
      end
    end

    function obj = overrideHeights(obj, override)
      obj.map_handle.overrideHeights(override);
    end

    function writeWRL(obj,fptr)
      error('not implemented yet, but could be done using the getAsMesh() interface');
    end

  end

  properties
    map_handle = [];
    default_normal_method;
  end
end
