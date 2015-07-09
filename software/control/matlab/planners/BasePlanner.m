classdef BasePlanner < DRCPlanner
  methods (Static)
    function obj = withAtlas()
      obj = BasePlanner(BasePlanner.constructAtlas());
    end

    function obj = withValkyrie(varargin)
      obj = BasePlanner(BasePlanner.constructValkyrie(varargin{:}));
    end
  end

  methods
    function obj = BasePlanner(varargin)
      obj = obj@DRCPlanner(varargin{:});
      obj = obj.addBaseSubscriptions();
    end
  end
end





