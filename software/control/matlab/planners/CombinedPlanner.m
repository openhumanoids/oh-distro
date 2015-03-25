classdef CombinedPlanner < BasePlanner & RemotePlanner
  methods (Static)
    function obj = withAtlas()
      obj = CombinedPlanner(CombinedPlanner.constructAtlas());
    end

    function obj = withValkyrie()
      obj = CombinedPlanner(CombinedPlanner.constructValkyrie());
    end
  end

  methods
    function obj = CombinedPlanner(varargin)
      obj = obj@BasePlanner(varargin{:});
    end
    
    function obj = addSubscriptions(obj)
      obj = addSubscriptions@BasePlanner(obj);
      obj = addSubscriptions@RemotePlanner(obj);
    end

  end
end





