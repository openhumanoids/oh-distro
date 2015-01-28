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
    function msg = to_iris_region_t(obj)
      msg = drc.iris_region_t();
      msg.lin_con = encodeLinCon(obj.A, obj.b);
      msg.point = obj.point;
      msg.normal = obj.normal;
    end

    function obj = from_iris_region_t(msg)
      A = reshape(msg.lin_con.A, msg.lin_con.m, msg.lin_con.n);
      b = msg.lin_con.b;
      point = msg.point;
      normal = msg.normal;
      obj = IRISRegion(A, b, [], [], point, normal);
    end
  end
end

