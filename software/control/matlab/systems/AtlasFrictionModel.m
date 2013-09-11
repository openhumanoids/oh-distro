classdef AtlasFrictionModel < FrictionModel
  methods
    % Construct a friction model with default gains
    function obj=AtlasFrictionModel(r)
      n = r.getNumInputs;
      g = zeros(7,n);
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'r_arm_usy')) = ...
        [9.9;20;0;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'r_arm_shx')) = ...
        [10.3;20;1.5;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'r_arm_ely')) = ...
        [9.6;14;1.7;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'r_arm_elx')) = ...
        [9.8;12;1.6;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'r_arm_uwy')) = ...
        [8.1;16;4.3;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'r_arm_mwx')) = ...
        [6.9;20;2.7;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'l_arm_usy')) = ...
        [10.2;17;0;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'l_arm_shx')) = ...
        [12.5;18;1.1;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'l_arm_ely')) = ...
        [11.2;14;1.3;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'l_arm_elx')) = ...
        [8.6;16;.8;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'l_arm_uwy')) = ...
        [9.0;11;1.3;.1];
      
      g(4:7,getStringIndices(r.getInputFrame.coordinates,'l_arm_mwx')) = ...
        [7.8;20;4.8;.1];
      
      obj = obj@FrictionModel(g,true);     
    end
  end
end