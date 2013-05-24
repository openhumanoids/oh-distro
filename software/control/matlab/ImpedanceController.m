classdef ImpedanceController < DrakeSystem
    % outputs a desired q_ddot
    properties
        nq;
        dt;
        Kp; % The proportional gain used in joint angle control
        Kd; % The derivative gain used in joint angle control
        Kk; % The stiffness used in impedance control
        Dk; % The damping used in impedance control
        controller_data;
        % pointer to shared data handle containing qtraj
        robot;
        lhand_ind;
        rhand_ind;
    end
    
    methods
        function obj = ImpedanceController(r,controller_data,options)
            typecheck(r,'Atlas');
            typecheck(controller_data,'SharedDataHandle');
            
            input_frame = r.getStateFrame;
            coords = AtlasCoordinates(r);
            obj = obj@DrakeSystem(0,0,input_frame.dim,coords.dim,true,true);
            obj = setInputFrame(obj,input_frame);
            obj = setOutputFrame(obj,coords);
            
            obj.controller_data = controller_data;
            obj.nq = getNumDOF(r);
            
            if nargin<3
                options = struct();
            end
            if isfield(options,'Kp')
              typecheck(options.Kp,'double');
              sizecheck(options.Kp,[obj.nq obj.nq]);
              obj.Kp = options.Kp;
            else
              obj.Kp = 170.0*eye(obj.nq);
              obj.Kp([1,2,6],[1,2,6]) = zeros(3); % ignore x,y,yaw
            end        

            if isfield(options,'Kd')
              typecheck(options.Kd,'double');
              sizecheck(options.Kd,[obj.nq obj.nq]);
              obj.Kd = options.Kd;
            else
              obj.Kd = 19.0*eye(obj.nq);
              obj.Kd([1,2,6],[1,2,6]) = zeros(3); % ignore x,y,yaw
            end  
            
            if isfield(options,'Kk')
              typecheck(options.Kk,'double');
              sizecheck(options.Kk,[12 12]) % change this in the future approximateIK supports quaternion
              if(any(real(eig(options.Kk))<-0.00001))
                error('The stiffness matrix Kk must be positive semidefinite');
              end
              obj.Kk = options.Kk;
            else
              obj.Kk = 100*eye(12);
            end
            
            if isfield(options,'Dk')
              typecheck(options.Dk,'double');
              sizecheck(options.Dk,[12 12]) % change this in the future approximateIK supports quaternion
              if(any(real(eig(options.Dk))<-0.00001))
                error('The damping matrix Dk must be positive semidefinite');
              end
              obj.Dk = options.Dk;
            else
              obj.Dk = 10*eye(12);
            end
            
            if isfield(options,'dt')
                typecheck(options.dt,'double');
                sizecheck(options.dt,[1 1]);
                obj.dt = options.dt;
            else
                obj.dt = 0.005;
            end
            
            obj.robot = r;
            obj.lhand_ind = obj.robot.findLinkInd('l_hand+l_hand_point_mass');
            obj.rhand_ind = obj.robot.findLinkInd('r_hand+r_hand_point_mass');
            obj = setSampleTime(obj,[obj.dt;0]);
        end
        
        function y = output(obj,t,~,x)
            q = x(1:obj.nq);
            qd = x(obj.nq+1:end);
            
            cdata = obj.controller_data.getData();
            
            if typecheck(cdata.qtraj,'double')
                q_des = cdata.qtraj;
            else
                q_des = cdata.qtraj.eval(t);
            end
            
            kinsol_des = doKinematics(obj.robot,q_des,false,true);
            % change the forwardKin to rotation_type = 2 when approximateIK
            % supports quaternion
            lhand_des = forwardKin(obj.robot,kinsol_des,obj.lhand_ind,[0;0;0],1);
            rhand_des = forwardKin(obj.robot,kinsol_des,obj.rhand_ind,[0;0;0],1);
            
            kinsol = doKinematics(obj.robot,q,false,true);
            [lhand_pos,Jl] = forwardKin(obj.robot,kinsol,obj.lhand_ind,[0;0;0],1);
            [rhand_pos,Jr] = forwardKin(obj.robot,kinsol,obj.rhand_ind,[0;0;0],1);
            
            err_q = q_des-q;
            y1 = obj.Kp*err_q - obj.Kd*qd;
            
            H = manipulatorDynamics(obj.robot,q,qd,true);
            err_x = [lhand_des-lhand_pos;rhand_des-rhand_pos];
            y2 = H\([Jl;Jr]'*(obj.Kk*err_x-obj.Dk*[Jl;Jr]*qd));
            
            y = y1+y2;
        end
    end
end