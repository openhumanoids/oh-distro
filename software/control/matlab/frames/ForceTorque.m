classdef ForceTorque < CoordinateFrame
  
  methods
    function obj=ForceTorque()

      % We could pass in a force torque sensors / generate one here
      % to extract coords, but is this ever going to change?
      coords{1}='force_x';
      coords{2}='force_y';
      coords{3}='force_z';
      coords{4}='torque_x';
      coords{5}='torque_y';
      coords{6}='torque_z';
      
      obj = obj@CoordinateFrame('ForceTorque',6,'f',coords);
    end
  end
end
