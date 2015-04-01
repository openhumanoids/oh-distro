classdef RemotePlanner < DRCPlanner
  methods (Static)
    function obj = withAtlas()
      obj = RemotePlanner(RemotePlanner.constructAtlas());
    end

    function obj = withValkyrie()
      obj = RemotePlanner(RemotePlanner.constructValkyrie());
    end
  end

  methods
    function obj = RemotePlanner(varargin)
      obj = obj@DRCPlanner(varargin{:});
      obj = obj.addRemoteSubscriptions();
    end
  end
end





