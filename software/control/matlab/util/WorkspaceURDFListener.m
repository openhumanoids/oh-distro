classdef WorkspaceURDFListener
  properties
    lc
    aggregator
  end
  
  methods
    function obj = WorkspaceURDFListener(channel)
      obj.lc = lcm.lcm.LCM.getSingleton();
      obj.aggregator = lcm.lcm.MessageAggregator();
      obj.lc.subscribe(channel,obj.aggregator);
    end
    
    function data = getNextMessage(obj,t_ms)
      msg = obj.aggregator.getNextMessage(t_ms);
      if(isempty(msg))
        data = [];
      else
        data = WorkspaceURDFListener.decode(drc.workspace_object_urdf_t(msg.data));
      end
    end
  end
  
  methods(Static)
    function data = decode(msg)
      urdf_string = char(msg.urdf_xml_string);
      data.urdf_file = [char(msg.otdf_type),num2str(msg.uid),'.urdf'];
      fid = fopen(data.urdf_file,'w');
      fprintf(fid,urdf_string);
      fclose(fid);
      data.uid = msg.uid;
    end
  end
end