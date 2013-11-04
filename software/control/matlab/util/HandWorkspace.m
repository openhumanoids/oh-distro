classdef HandWorkspace
  % This constructs/loads a works space of the hand, and can query the hand posture that
  % give the end effector pose closest to the desired posture
  properties
    workspace_mat_path
    l_hand_pos; % The relative pose of l_hand to utorso
    r_hand_pos; % The relative pose of r_hand to utorso
    l_arm_samples;
    r_arm_samples
    n_samples
  end
  
  methods
    function obj = HandWorkspace(path)
      if(exist(path,'file'))
        mat_data = load(path);
        obj.workspace_mat_path = path;
        obj.l_hand_pos = mat_data.l_hand_pos;
        obj.r_hand_pos = mat_data.r_hand_pos;
        obj.l_arm_samples = mat_data.l_arm_samples;
        obj.r_arm_samples = mat_data.r_arm_samples;
        obj.n_samples = mat_data.n_samples;
      else
        obj = constructWorkSpace(obj,path);
      end
    end
    
    function obj = constructWorkSpace(obj,path)
      options.floating = true;
      robot = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact.urdf'),options);
      nomdata = load([getenv('DRC_PATH'),'/control/matlab/data/aa_atlas_fp.mat']);
      nq = robot.getNumDOF();
      qstar = nomdata.xstar(1:nq);
      coords = robot.getStateFrame.coordinates;
      coords = coords(1:nq);
      l_arm_joints = cellfun(@(s) ~isempty(strfind(s,'l_arm')),coords);
      r_arm_joints = cellfun(@(s) ~isempty(strfind(s,'r_arm')),coords);
      l_hand = robot.findLinkInd('l_hand');
      r_hand = robot.findLinkInd('r_hand');
      utorso = robot.findLinkInd('utorso');
      l_clav = robot.findLinkInd('l_clav');
      r_clav = robot.findLinkInd('r_clav');
      kinsol = doKinematics(robot,qstar);
      l_clav_pts = [0;0;0];
      r_clav_pts = [0;0;0];
      utorso_pts = [0;0;0];
      utorso_pos = forwardKin(robot,kinsol,utorso,utorso_pts,2);
      l_hand_pts = [0;0;0];
      r_hand_pts = [0;0;0];
      num_l_arm_joints = sum(l_arm_joints);
      num_r_arm_joints = sum(r_arm_joints);
      [joint_lb,joint_ub] = robot.getJointLimits();
      l_arm_lb = joint_lb(l_arm_joints);
      l_arm_ub = joint_ub(l_arm_joints);
      r_arm_lb = joint_lb(r_arm_joints);
      r_arm_ub = joint_ub(r_arm_joints);
      obj.n_samples = 20;
      obj.l_arm_samples = repmat(l_arm_lb,1,obj.n_samples)+(l_arm_ub-l_arm_lb)/(obj.n_samples+2)*(1:obj.n_samples);
      obj.r_arm_samples = repmat(r_arm_lb,1,obj.n_samples)+(r_arm_ub-r_arm_lb)/(obj.n_samples+2)*(1:obj.n_samples);
      obj.l_hand_pos = zeros(7,obj.n_samples^num_l_arm_joints);
      obj.r_hand_pos = zeros(7,obj.n_samples^num_r_arm_joints);
      for i = 1:obj.n_samples
        for j = 1:obj.n_samples
          for k = 1:obj.n_samples
            for l =1:obj.n_samples
              for m = 1:obj.n_samples
                for n = 1:obj.n_samples
                  ind = power(obj.n_samples,5:-1:0)*([i;j;k;l;m;n]-1)+1;
                  q_sample = qstar;
                  q_sample(l_arm_joints) = [obj.l_arm_samples(1,i);obj.l_arm_samples(2,j);obj.l_arm_samples(3,k);obj.l_arm_samples(4,l);obj.l_arm_samples(5,m);obj.l_arm_samples(6,n)];
                  q_sample(r_arm_joints) = [obj.r_arm_samples(1,i);obj.r_arm_samples(2,j);obj.r_arm_samples(3,k);obj.r_arm_samples(4,l);obj.r_arm_samples(5,m);obj.r_arm_samples(6,n)];
                  kinsol = robot.doKinematics(q_sample);
                  obj.l_hand_pos(:,ind) = forwardKin(robot,kinsol,l_hand,l_hand_pts,2);
                  obj.r_hand_pos(:,ind) = forwardKin(robot,kinsol,r_hand,r_hand_pts,2);
                end
              end
            end
          end
        end
      end
      obj.l_hand_pos(1:3,:) = obj.l_hand_pos(1:3,:)-bsxfun(@times,utorso_pos(1:3),ones(1,obj.n_samples^num_l_arm_joints));
      obj.r_hand_pos(1:3,:) = obj.r_hand_pos(1:3,:)-bsxfun(@times,utorso_pos(1:3),ones(1,obj.n_samples^num_r_arm_joints));
      obj.l_hand_pos(4:7,:) = (quatmultiply(obj.l_hand_pos(4:7,:)',[utorso_pos(4) -utorso_pos(5) -utorso_pos(6) -utorso_pos(7)]))';
      obj.r_hand_pos(4:7,:) = (quatmultiply(obj.r_hand_pos(4:7,:)',[utorso_pos(4) -utorso_pos(5) -utorso_pos(6) -utorso_pos(7)]))';
      l_hand_pos = obj.l_hand_pos;
      r_hand_pos = obj.r_hand_pos;
      l_arm_samples = obj.l_arm_samples;
      r_arm_samples = obj.r_arm_samples;
      n_samples = obj.n_samples;
      save(path,'l_hand_pos','r_hand_pos','l_arm_samples','r_arm_samples','n_samples');
      obj.workspace_mat_path = path;
    end
    
    function q_arm = closestSample(obj,utorso_pos,hand_pos,leftHand)
      hand_rel = [hand_pos(1:3)-utorso_pos(1:3);quatmultiply(hand_pos(4:7)',[utorso_pos(4) -utorso_pos(5) -utorso_pos(6) -utorso_pos(7)])'];
      if(leftHand)
        hand_sample_pos = obj.l_hand_pos;
      else
        hand_sample_pos = obj.r_hand_pos;
      end
      hand_pos_diff = bsxfun(@times,hand_rel(1:3),ones(1,size(hand_sample_pos,2)))-hand_sample_pos(1:3,:);
      hand_quat_diff = 1-(hand_rel(4:7)'*hand_sample_pos(4:7,:)).^2;
      hand_dist = sum(hand_pos_diff.^2,1)+1*hand_quat_diff;
      [~,hand_min_idx] = min(hand_dist);
      i = floor((hand_min_idx-1)/obj.n_samples^5)+1;
      j = floor((hand_min_idx-(i-1)*obj.n_samples^5-1)/obj.n_samples^4)+1;
      k = floor((hand_min_idx-(i-1)*obj.n_samples^5-(j-1)*obj.n_samples^4-1)/obj.n_samples^3)+1;
      l = floor((hand_min_idx-(i-1)*obj.n_samples^5-(j-1)*obj.n_samples^4-(k-1)*obj.n_samples^3-1)/obj.n_samples^2)+1;
      m = floor((hand_min_idx-(i-1)*obj.n_samples^5-(j-1)*obj.n_samples^4-(k-1)*obj.n_samples^3-(l-1)*obj.n_samples^2-1)/obj.n_samples^1)+1;
      n = floor((hand_min_idx-(i-1)*obj.n_samples^5-(j-1)*obj.n_samples^4-(k-1)*obj.n_samples^3-(l-1)*obj.n_samples^2-(m-1)*obj.n_samples-1)/obj.n_samples^0)+1;
      if(leftHand)
        q_arm = [obj.l_arm_samples(1,i);obj.l_arm_samples(2,j);obj.l_arm_samples(3,k);obj.l_arm_samples(4,l);obj.l_arm_samples(5,m);obj.l_arm_samples(6,n)];
      else
        q_arm = [obj.r_arm_samples(1,i);obj.r_arm_samples(2,j);obj.r_arm_samples(3,k);obj.r_arm_samples(4,l);obj.r_arm_samples(5,m);obj.r_arm_samples(6,n)];
      end
    end
  end
end