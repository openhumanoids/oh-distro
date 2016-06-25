classdef CombinedPlanner < DRCPlanner
  methods (Static)
    function obj = withAtlas()
      obj = CombinedPlanner(CombinedPlanner.constructAtlas());
    end

    function obj = withValkyrie()
      obj = CombinedPlanner(CombinedPlanner.constructValkyrie(2));
    end
  end

  methods
    function obj = CombinedPlanner(varargin)
      obj = obj@DRCPlanner(varargin{:});
      obj = obj.addRemoteSubscriptions();
      obj = obj.addBaseSubscriptions();
    end
  end
end





