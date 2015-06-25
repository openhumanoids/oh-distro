classdef MyRigidBodyFlatTerrain < RigidBodyFlatTerrain
  methods 
    function obj = MyRigidBodyFlatTerrain(varargin)
      obj = obj@RigidBodyFlatTerrain(varargin{:});
    end
    
    function geom = constructRigidBodyGeometry(obj)
      box_width = 1;
      box_depth = 0.01;
      geom = RigidBodyBox([box_width;box_width;box_depth]);
      geom.T(3,4) = obj.z - box_depth/2;
      geom.c = hex2dec({'ee','cb','ad'})'/256;  % something a little brighter (peach puff 2 from http://www.tayloredmktg.com/rgb/);
      geom.name = 'terrain';
%      geom.c = hex2dec({'cd','af','95'})'/256;
    end
  end
end