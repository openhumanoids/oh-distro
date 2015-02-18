classdef IRISRegion < iris.TerrainRegion

  methods
    function obj = IRISRegion(varargin)
      obj = obj@iris.TerrainRegion(varargin{:});
    end

    function msg = toLCM(obj)
      msg = obj.to_iris_region_t();
    end
  end

  methods (Static=true)
    function msg = to_iris_region_t(obj, yaw)
      if nargin < 2
        yaw = 0;
      end
      msg = drc.iris_region_t();
      msg.lin_con = encodeLinCon(obj.A, obj.b);
      ax = cross([0;0;1], obj.normal);
      if norm(ax) < 1e-4
        ax = [0;0;1];
        ang = 0;
      else
        ang = asin(norm(ax) / norm(obj.normal));
      end
      T = makehgtform('zrotate', yaw, 'axisrotate', ax, ang, 'translate', obj.point);
      msg.seed_pose = encodePosition3d([T(1:3,4); rotmat2rpy(T(1:3,1:3))]);
    end

    function obj = from_iris_region_t(msg)
      A = reshape(msg.lin_con.A, msg.lin_con.m, msg.lin_con.n);
      b = msg.lin_con.b;
      pose = decodePosition3d(msg.seed_pose);
      point = pose(1:3);
      normal = rpy2rotmat(pose(4:6)) * [0;0;1];
      obj = IRISRegion(A, b, [], [], point, normal);
    end
  end
end

