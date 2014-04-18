classdef FootContactState < CoordinateFrame
  methods
    function obj=FootContactState()
      obj = obj@CoordinateFrame('FootContactState',2,'x',{'left','right'});
    end
  end
end
