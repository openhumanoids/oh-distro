classdef SignalDuplicator < MIMODrakeSystem

  methods
  function obj = SignalDuplicator(frame,n_outputs)
    typecheck(frame,'CoordinateFrame');
    rangecheck(n_outputs,1,inf);
    
    input_frame = frame; 
    outs = cell(n_outputs,1);
    for i=1:n_outputs
      outs{i} = frame;
    end
    output_frame = MultiCoordinateFrame(outs);
    
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);
  
    obj.n_outputs = n_outputs;
  end
    
  function varargout=mimoOutput(obj,~,~,input)
    varargout = cell(obj.n_outputs,1);
    for i=1:obj.n_outputs
      varargout{i} = input;
    end
  end
  end

  properties
    n_outputs;
  end
end
