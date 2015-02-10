classdef MassEstimationBlock < MIMODrakeSystem
  
  properties
    lc; % LCM
    broadcast_channel;
    r_hand_joint_inds;
    
    % robot
    r;
    r_control;
    
    % frame of ft sensor
    ft_frame;
    
  end
  
  methods
    function obj = MassEstimationBlock(r,r_control,ft_frame,options)
      typecheck(r,'TimeSteppingRigidBodyManipulator');
      if (nargin >= 2 && ~isempty(r_control))
        typecheck(r_control, 'TimeSteppingRigidBodyManipulator');
      else
        r_control = r;
      end
      if nargin<4
        options = struct();
      end
      
      output_frame = getOutputFrame(r);
      obj = obj@MIMODrakeSystem(0,0,output_frame,output_frame,true,true);
      obj = setInputFrame(obj,output_frame);
      obj = setOutputFrame(obj,output_frame);
      if isfield(options,'dt')
        typecheck(options.dt,'double');
        sizecheck(options.dt,[1 1]);
        dt = options.dt;
      else
        dt = 0.001;
      end
      obj = setSampleTime(obj,[dt;0]); % sets controller update rate
      
      % Get LCM set up for broadcast on approp channels
      obj.lc = lcm.lcm.LCM.getSingleton();
      
      obj.r = r;
      obj.r_control = r_control;
      obj.ft_frame = ft_frame;
    end
    
    function varargout=mimoOutput(obj,t,~,varargin)
      robot_state = varargin{1};
      tsmanip = obj.r;
      manip = obj.r.getManipulator();
      
      % And contact stuff
      z = tsmanip.LCP_cache.data.z;
      if (~isempty(z))
        z = z/tsmanip.timestep;
        
        kinsol = doKinematics(manip,tsmanip.LCP_cache.data.x(1:tsmanip.getNumPositions));
        % Generate simulated readings from the force-torque sensor
        % if simulation is running
        
        % Mass of child links
        children = [obj.ft_frame.body_ind];
        for i=1:length(manip.body)
          if (i ~= obj.ft_frame.body_ind)
            body = manip.body(i);
            if ~isempty(find(children==body.parent, 1))
              children = [children i];
            end
          end
        end
        xdn_masses = zeros(6, 1);
        sensor_posquat = forwardKin(manip,kinsol,findFrameId(manip,obj.ft_frame.name),zeros(3,1), 2);
        sensor_pos = sensor_posquat(1:3);
        sensor_quat = sensor_posquat(4:7);
        g_rotated = quatrotate(sensor_quat.', [0 0 -9.8]).';
        for i=1:length(children)
          if children(i) ~= obj.ft_frame.body_ind
            body = manip.body(children(i));
            m = body.mass;
            com = body.com;
            com_pos = forwardKin(manip,kinsol,children(i),com);
            % Exerts g*m straight down; rotate into sensor frame
            % and exert as linear force and r cross f
            mg = m*g_rotated;
            xdn_masses(1:3) = xdn_masses(1:3) + mg;
            torque = cross(com_pos-sensor_pos, mg);
            xdn_masses(4:6) = xdn_masses(4:6) + torque;
            
          end
        end
        
        % borrowed from ContactForceTorqueSensor, except I look at all
        % contact everywhere
        contact_link_idxA = tsmanip.LCP_cache.data.contact_data.idxA;
        contact_link_idxB = tsmanip.LCP_cache.data.contact_data.idxB;
        % prune out those relating to bodies we want to ignore
        % -- for now none of them, eventually we only want children of
        % link of interest (but at that point we're writing a real FT
        % sensor and not my quick hack here)
        contact_idxA = [];
        contact_idxB = [];
        for i=1:length(contact_link_idxA)
          ind = contact_link_idxA(i);
          % Follow parents until we either reach our own target link, 
          % or hit 0 (which means it's not our child)
          parent = ind;
          while (parent ~= 0)
            if (parent == obj.ft_frame.body_ind)
              contact_idxA = [contact_idxA contact_link_idxA(i)];
              parent = 0;
            end
            parent = manip.body(parent).parent;
          end
        end
        for i=1:length(contact_link_idxB)
          ind = contact_link_idxB(i);
          % Follow parents until we either reach our own target link, 
          % or hit 0 (which means it's not our child)
          parent = ind;
          while (parent ~= 0)
            if (parent == obj.ft_frame.body_ind)
              contact_idxB = [contact_idxB contact_link_idxB(i)];
              parent = 0;
            end
            parent = manip.body(parent).parent;
          end
        end
        
        if isempty(contact_idxA)
          contact_idxA = zeros(1,0);
        end
        if isempty(contact_idxB)
          contact_idxB = zeros(1,0);
        end
        nA = length(contact_idxA);
        nB = length(contact_idxB);
        N = nA + nB;
        if (N>0)
          % extract relevant contact information
          % contact positions on the body
          contact_pos_body = [tsmanip.LCP_cache.data.contact_data.xA(:,contact_idxA) ...
            tsmanip.LCP_cache.data.contact_data.xB(:,contact_idxB)];
          
          % contact normal and tangential directions in world coordinates
          % (note that they could be different sizes on different calls, so
          % check the cache elements to extract the sizes)
          nD = length(tsmanip.LCP_cache.data.contact_data.d);
          nC_body = nA + nB;
          nC = size(tsmanip.LCP_cache.data.contact_data.normal,2);
          nL = sum(tsmanip.LCP_cache.data.possible_limit_indices);
          nP = manip.num_position_constraints;  % number of position constraints
          
          normal_world = [tsmanip.LCP_cache.data.contact_data.normal(:,contact_idxA)...
            -tsmanip.LCP_cache.data.contact_data.normal(:,contact_idxB)];
          
          d_mat = cell2mat(tsmanip.LCP_cache.data.contact_data.d);
          tangent_world = [d_mat(:,kron(0:nD-1,nC*ones(1,nA)) + repmat(contact_idxA,1,nD)) ...
            -d_mat(:,kron(0:nD-1,nC*ones(1,nB)) + repmat(contact_idxB,1,nD))];
          
          sensor_pos = forwardKin(manip,kinsol,findFrameId(manip,obj.ft_frame.name),zeros(3,1));
          normal = bodyKin(manip,kinsol,findFrameId(manip,obj.ft_frame.name),repmat(sensor_pos,1,N)+normal_world);
          tangent = bodyKin(manip,kinsol,findFrameId(manip,obj.ft_frame.name),repmat(sensor_pos,1,N*nD)+tangent_world);
          tangent = [tangent -tangent];
          
          normal_ind = nL+nP+[contact_idxA contact_idxB];
          tangent_ind = [kron(0:nD-1,nC*ones(1,nA)) + repmat(contact_idxA,1,nD) ...
            kron(0:nD-1,nC*ones(1,nB)) + repmat(contact_idxB,1,nD)];
          
          tangent_ind = nL+nP+nC + [tangent_ind (tangent_ind + nD*nC)];
          
          % compute all individual contact forces in sensor coordinates
          
          force = normal*z(normal_ind) + tangent*z(tangent_ind);
          
          torque = zeros(3,1);
          for i=1:N,
            torque = torque + cross(contact_pos_body(:,i),normal(:,i)*z(normal_ind(i)));
            for j=1:2*nD,
              torque = torque + cross(contact_pos_body(:,i),tangent(:,i+(j-1)*N)*z(tangent_ind(i+(j-1)*N)));
            end
          end
          
          xdn = [force;torque];
        else
          xdn = zeros(6,1);
        end
        
        % compose as an LCM message and broadcast
        % we'll just overload the DRC force_torque message type for now...
        force_torque_msg = drc.force_torque_t();
        
        % pack it up
        ft_total = xdn_masses; %xdn+xdn_masses;
        force_torque_msg.r_hand_force = ft_total(1:3); 
        force_torque_msg.r_hand_torque = ft_total(4:6);
        obj.lc.publish('GIZATT_FORCE_TORQUE_NSTC', force_torque_msg);
        
        
        
        % and now that we finally have FT info...
        % We have forces and torques at what we'll call a final link
        % (in the Atkeson '86 sense)
        % We need to generate A
        % We need the acceleration of our target link first
        
      end
      
      varargout = varargin;
    end
  end
end
