classdef DRCStateMachine

  properties (SetAccess=private,GetAccess=public)
    active_controller;
    controllers; 
  end

  methods
    function obj = DRCStateMachine(controllers,active_controller)
      typecheck(controllers,'struct');
      obj.controllers = controllers;
      obj = setActiveController(obj,active_controller);
    end
    
    function obj = addController(obj,name,controller)
      typecheck(name,'char');
      typecheck(controller,'DRCController');
      obj.controllers.(name) = controller;
    end
    
    function obj = setActiveController(obj,active_controller)
      typecheck(active_controller,'char');
      if ~any(strcmp(active_controller,fieldnames(obj.controllers)))
        error(strcmp('DRCStateMachine::Invalid controller name:',active_controller));
      end
      obj.active_controller = active_controller;
    end
    
    function run(obj)
      data = [];
      transition_tic = tic;
      while 1
        ctrl = obj.controllers.(obj.active_controller);
        msg = ['Controller: initializing ' ctrl.name];
        send_status(3, 0, 0, msg );
        disp(msg);
        
        init_tic = tic;
        ctrl = ctrl.initialize(data);
        disp([ctrl.name ' initialize time: ' num2str(toc(init_tic))]);
        
        msg = ['Controller: running ' ctrl.name];
        send_status(3, 0, 0, msg );
        send_controller_status(ctrl.name);
        disp(msg);
        disp(['Controller: transition time ' num2str(toc(transition_tic))]);
        transition_data = ctrl.run();
        transition_tic = tic;
        fn = fieldnames(transition_data);
        transition_to = fn{1}; % arbitrarily take the first one if multiple transitions occured simultaneously
        
        msg = ['Controller: transitioning from ' ctrl.name ' to ' transition_to];
        send_status(3, 0, 0, msg );
        disp(msg);
        
        obj = setActiveController(obj,transition_to);
        data = transition_data.(obj.active_controller);
      end
    end
  end
  
end
