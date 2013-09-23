classdef SignalDuplicator < MIMODrakeSystem

  methods
  function obj = SignalDuplicator(frame,n_outputs)

    typecheck(frame,'CoordinateFrame');
    rangecheck(n_outputs,1,inf);
    
    input_frame = frame; 
    outs = cell(n_outputs);
    B=zeros(n_outputs*frame.dim,frame.dim);
    for i=1:n_outputs
      outs{i} = frame;
      B((i-1)*frame.dim+(1:frame.dim),:) = eye(frame.dim);
    end
    output_frame = MultiCoordinateFrame(outs);
    
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);
  
    obj.B = B;
  end
    
  function y=mimoOutput(obj,~,~,input)
    y = obj.B*input;
  end
  end

  properties
    B;
  end
end
