classdef IRISRegionList
  properties
    iris_regions
    region_id
  end

  methods
    function obj = IRISRegionList(iris_regions, ids)
      obj.iris_regions = iris_regions;
      obj.region_id = ids;
    end

    function msg = to_iris_region_response_t(obj)
      msg = drc.iris_region_response_t();
      msg.num_iris_regions = length(obj.iris_regions);
      region_msgs = javaArray('drc.iris_region_t', msg.num_iris_regions);
      for j = 1:msg.num_iris_regions
        region_msgs(j) = IRISRegion.to_iris_region_t(obj.iris_regions(j));
      end
      msg.iris_regions = region_msgs;
      msg.region_id = obj.region_id;
    end

    function msg = toLCM(obj)
      msg = obj.to_iris_region_response_t();
    end
  end
end
