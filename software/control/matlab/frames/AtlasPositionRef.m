classdef AtlasPositionRef < LCMCoordinateFrameWCoder & Singleton
  % atlas position reference input frame
  methods
    function obj=AtlasPositionRef(r,mode,gains_id)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      if (nargin<2) 
        mode=2; 
      end
      
      rangecheck(mode,2,4);
      
      nq=getNumInputs(r);
      if (mode==2) dim=nq;
      elseif (mode==3) dim=2*nq;
      elseif (mode==4) dim=3*nq;
      end
      
      if nargin<3 % controlling robot
        warning('AtlasPositionRef: USING ATLAS GAINS')
        gains = getAtlasGains(r,mode);
      else
        warning('AtlasPositionRef: USING SIMULATION GAINS')
        typecheck(gains_id,'char');
        gains = struct();
        gains.k_qd_p = zeros(nq,1);
        gains.k_q_i = zeros(nq,1);
        gains.k_f_p = zeros(nq,1);
        gains.ff_f_d = zeros(nq,1);
        gains.ff_qd_d = zeros(nq,1);
        gains.ff_const = zeros(nq,1);

        [Kp,Kd] = getPDGains(r,gains_id);
        gains.k_q_p = diag(Kp);
        gains.ff_qd = diag(Kd);
      end

      obj = obj@LCMCoordinateFrameWCoder('AtlasPositionRef',dim,'x');
      obj = obj@Singleton(['AtlasPositionRef_sendmode=',num2str(mode)]);
      if isempty(obj.lcmcoder)
        input_names = r.getInputFrame().coordinates;
        input_names = regexprep(input_names,'_motor',''); % remove motor suffix
      
        coder = drc.control.AtlasCommandCoder(input_names,2,gains.k_q_p,gains.k_q_i,...
          gains.k_qd_p,gains.k_f_p,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d,gains.ff_const);
        obj = setLCMCoder(obj,JLCMCoder(coder));
        
        coords = input_names;
        if (mode>2)
          coords = vertcat(coords,cellfun(@(a) [a,'_dot'],input_names,'UniformOutput',false));
        end
        if (mode>3)
          coords = vertcat(coords,cellfun(@(a) [a,'_ff'],input_names,'UniformOutput',false));
        end
        obj.setCoordinateNames(coords);
        obj.setDefaultChannel('ATLAS_COMMAND');
      end
      
      if (obj.mex_ptr==0)
        obj.mex_ptr = AtlasCommandPublisher(input_names,2,gains.k_q_p,gains.k_q_i,...
          gains.k_qd_p,gains.k_f_p,gains.ff_qd,gains.ff_qd_d,gains.ff_f_d,gains.ff_const);
        obj = setLCMCoder(obj,JLCMCoder(coder));
      end
    end
    
%    function publish(obj,t,x,channel)
%      % short-cut java publish with a faster mex version
%      AtlasCommandPublisher(obj.mex_ptr,channel,t,x);
%    end
%    
%    function delete(obj)
%      AtlasCommandPublisher(obj.mex_ptr);
%    end
  end
  
  properties
    mex_ptr=0
  end
end
