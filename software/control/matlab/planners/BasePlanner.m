classdef BasePlanner < DRCPlanner
  methods (Static)
    function obj = withAtlas()
      obj = BasePlanner(BasePlanner.constructAtlas());
    end

    function obj = withValkyrie()
      obj = BasePlanner(BasePlanner.constructValkyrie());
    end
  end

  methods
    function obj = BasePlanner(varargin)
      obj = obj@DRCPlanner(varargin{:});
      obj = obj.addBaseSubscriptions();
    end
  end
end





