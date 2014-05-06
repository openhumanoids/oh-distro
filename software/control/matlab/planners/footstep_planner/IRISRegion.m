classdef IRISRegion
  properties
    A
    b
    point
    normal
  end

  methods
    function obj = IRISRegion(A, b, point, normal)
      obj.A = A;
      obj.b = b;
      obj.point = point;
      obj.normal = normal;
    end

    function msg = to_iris_region_t(obj)
      msg = drc.iris_region_t();
      msg.lin_con = drc.lin_con_t();
      msg.lin_con.m = size(obj.A, 1);
      msg.lin_con.n = size(obj.A, 2);
      msg.lin_con.m_times_n = msg.lin_con.m*msg.lin_con.n;
      msg.lin_con.A = reshape(obj.A, [], 1);
      msg.lin_con.b = obj.b;
      msg.point = obj.point;
      msg.normal = obj.normal;
    end
  end

  methods (Static=true)
    function obj = from_iris_region_t(msg)
      A = reshape(msg.lin_con.A, msg.lin_con.m, msg.lin_con.n);
      b = msg.lin_con.b;
      point = msg.point;
      normal = msg.normal;
      obj = IRISRegion(A, b, point, normal);
    end
  end
end

