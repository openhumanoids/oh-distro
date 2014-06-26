classdef QPControllerData < handle
  % Class that contains data needed by the QPController.
  % This is a handle class because some of these quantities change during
  % execution. 

  % private SetAccess properties change infrequently
  properties (SetAccess=private,GetAccess=public)
    % ZMP dynamics ---------------------------------------------------------------
    A = [zeros(2),eye(2); zeros(2,4)] % COM state map
    B = [zeros(2); eye(2)] % COM input map
    C = [eye(2),zeros(2)] % ZMP state-output map
    D % ZMP input-output map

    % dynamics related -----------------------------------------------------------
    mu; % friction coefficient    
  end
  
  % public SetAccess properties can change after instantiation
  properties (SetAccess=public,GetAccess=public)
    % solver related -------------------------------------------------------------
    infocount=0 % number of consecutive iterations with solver info < 0
    qp_active_set=[]% active set of inequality constraints from pervious iteration
    
    % ZMP-LQR terms --------------------------------------------------------------
    x0 % nominal COM state: [x_com;y_com;xd_com;yd_com], typically centered 
    % between feet for standing, or at end of walking trajectory
    y0 % nominal ZMP: [x_zmp;y_zmp]
    u0=zeros(2,1) % nominal input: [xdd_com;ydd_com]
    Qy % ZMP output LQR cost
    R=zeros(2) % ZMP input cost
    S % cost-to-go terms: x'Sx + x's1 + s2
    s1
    s2
    Sdot % cost-to-go derivatives
    s1dot
    s2dot
    is_time_varying

    % motion planning ------------------------------------------------------------
    qtraj % generalize configuration vector or trajectory 
    comtraj % COM state trajectory
    support_times % vector of contact transition times
    supports % (array of) SupportState(s)
    ignore_terrain %
    link_constraints=[] % structure of link motion constraints, see Biped class
    constrained_dofs=[] % array of joint indices
    plan_shift=[0;0;0] % linear translation to apply to walking plan (applied to 
    % COM and ZMP trajectories)  
    
    acceleration_input_frame; % input coordinate frame for desired 
    % generalized accelerations
    optional_data=[]; % catch-all struct for other data that you might want to pass
  end
  
  methods 
    function obj = QPControllerData(is_time_varying,data)
      typecheck(is_time_varying,'logical');
      typecheck(data,'struct');
  
      obj.is_time_varying = is_time_varying;
      obj=checkAndUpdateQPData(obj,data);
    end
 
    function obj = checkAndUpdateQPData(obj,data)
      
      assert(isa(data.acceleration_input_frame,'CoordinateFrame'));
      obj.acceleration_input_frame = data.acceleration_input_frame;
      
      assert(isa(data.qtraj,'Trajectory') || isnumeric(data.qtraj));
      obj.qtraj = data.qtraj;
      
      if obj.is_time_varying
        assert(isa(data.comtraj,'Trajectory'));
        obj.comtraj = data.comtraj;
        assert(isa(data.y0,'Trajectory'));
        assert(isa(data.s1,'Trajectory'));
        assert(isa(data.s2,'Trajectory'));
        if isfield(data,'s1dot')
          assert(isa(data.s1dot,'Trajectory'));
        else
          data.s1dot = fnder(data.s1,1);
        end
        obj.s1dot = data.s1dot;

        if isfield(data,'s2dot')
          assert(isa(data.s2dot,'Trajectory'));
        else
          data.s2dot = fnder(data.s2,1);
        end
        obj.s2dot = data.s2dot;
      else
        assert(isnumeric(data.y0));
        assert(isnumeric(data.s1));
        assert(isnumeric(data.s2));
      end
      
      obj.supports = data.supports;
      assert(isnumeric(data.support_times));
      obj.support_times = data.support_times;

      assert(isnumeric(data.x0));
      sizecheck(data.x0,[4 1]);
      obj.x0 = data.x0;
        
      assert(isnumeric(data.mu));
      obj.mu = data.mu;

      assert(islogical(data.ignore_terrain));
      obj.ignore_terrain = data.ignore_terrain;

      assert(isnumeric(data.D));
      sizecheck(data.D,[2 2]);
      obj.D = data.D;
      
      assert(isnumeric(data.Qy));
      obj.Qy = data.Qy;       
      
      assert(isnumeric(data.S));
      sizecheck(data.S,[4 4]);
      obj.S = data.S;
      
      sizecheck(data.s1,[4 1]);
      sizecheck(data.s2,1);
      obj.s1 = data.s1;
      obj.s2 = data.s2;
      obj.y0 = data.y0;
      
      % optional properties
      if isfield(data,'qp_active_set')
        obj.qp_active_set = data.qp_active_set;
      end
      if isfield(data,'link_constraints')
        assert(isstruct(data.link_constraints));
        obj.link_constraints = data.link_constraints;
      end
      if isfield(data,'constrained_dofs')
        obj.constrained_dofs = data.constrained_dofs;
      end
      if isfield(data,'R')
        assert(isnumeric(data.R));
        sizecheck(data.R,[2 2]);
        obj.R = data.R;
      end
      if isfield(data,'C')
        assert(isnumeric(data.C));
        obj.C = data.C;
      end
      if isfield(data,'u0')
        assert(isnumeric(data.u0));
        obj.u0 = data.u0;
      end
      if isfield(data,'optional_data')
        assert(isstruct(data.optional_data));
        obj.optional_data = data.optional_data;
      end

    end
  end
end