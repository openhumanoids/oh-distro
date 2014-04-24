r = RigidBodyManipulator(strcat(getenv('DRC_PATH'),'/models/mit_gazebo_models/mit_robot_drake/model_minimal_contact_point_hands.urdf'),struct('floating',true'));
joint_indices = [22:26 33]; % right arm
body = 29; % right hand
q0 = zeros(34,1);
[lb,ub] = r.getJointLimits;
%%
N = 15;
if 0
tic
q_range = zeros(6,N);
for i=1:6,
  q_range(i,:) = linspace(lb(joint_indices(i)),ub(joint_indices(i)),N);
end
x = zeros(N,N,N,N,N,N,7);
for i=1:N,
  q0(joint_indices(1)) = q_range(1,i);
  for j=1:N,
    q0(joint_indices(2)) = q_range(2,j);
    for k=1:N,
      q0(joint_indices(3)) = q_range(3,k);
      for l=1:N,
        q0(joint_indices(4)) = q_range(4,l);
        for m=1:N,
          q0(joint_indices(5)) = q_range(5,m);
          for n=1:N,
            q0(joint_indices(6)) = q_range(6,n);
% 						tic;
						kinsol = r.doKinematics(q0);
            x(i,j,k,l,m,n,:) = r.forwardKin(kinsol,body,[0;-.135;0],2);
% 						toc;
					end
        end
      end
    end
  end
  i
end
toc
save(sprintf('fk_data_x_N%d.mat',N),'x');
else
load(sprintf('fk_data_x_N%d.mat',N));
	
end

%%
pos=reshape(x(1:N,1:N,1:N,1:N,1:N,1:N,1:3),[],3);
orient = reshape(x(1:N,1:N,1:N,1:N,1:N,1:N,4:end),[],4);
pos_max = max(pos);
pos_min = min(pos);
M = 20;
x_range = linspace(pos_min(1),pos_max(1),M+1);
y_range = linspace(pos_min(2),pos_max(2),M+1);
z_range = linspace(pos_min(3),pos_max(3),M+1);
% score = zeros(M,M,M);

%%
score = zeros(M,M,M);
orient_list = cell(M,M,M);
x_range(end) = x_range(end) + 1e-6;
y_range(end) = y_range(end) + 1e-6;
z_range(end) = z_range(end) + 1e-6;
for i=1:length(pos),
  x_ind = find(pos(i,1) < x_range,1)-1;
  y_ind = find(pos(i,2) < y_range,1)-1;
  z_ind = find(pos(i,3) < z_range,1)-1;
  score(x_ind,y_ind,z_ind) = score(x_ind,y_ind,z_ind) + 1;
  if isempty(orient_list{x_ind,y_ind,z_ind})
    orient_list{x_ind,y_ind,z_ind} = orient(i,:);
  else
    if max(orient_list{x_ind,y_ind,z_ind}*orient(i,:)') < .95
      orient_list{x_ind,y_ind,z_ind} = [orient_list{x_ind,y_ind,z_ind};orient(i,:)];
    end
  end
  if rem(i,1e6) == 1
    i
  end
end
%%
% (.4, -.3 ,.5) roughly in center of view
x_score = zeros(M,M,M);
y_score = zeros(M,M,M);
z_score = zeros(M,M,M);
for i=1:M,
  x_score(i,:,:) = (x_range(i) + x_range(i+1))/2*ones(M,M);
  y_score(:,i,:) = (y_range(i) + y_range(i+1))/2*ones(M,M);
  z_score(:,:,i) = (z_range(i) + z_range(i+1))/2*ones(M,M);
end


orient_score = cellfun(@(x) size(x,1),orient_list(:));
score = score(:);
x_score = x_score(:);
y_score = y_score(:);
z_score = z_score(:);
I = find(score);
% score_sub = score(I);
% orient_score = orient_score(I);
% x_score = x_score(I);
% y_score = y_score(I);
% z_score = z_score(I);
% scatter3(x_score(:),y_score(:),z_score(:),[],repmat([0 0 1], length(score),1) + score(:)*[1 0 -1]/max(max(max(score))))
% scatter3(x_score(:),y_score(:),z_score(:),[],repmat([0 0 1], ...
%   length(score_sub),1) + orient_score(:)*[1 0 -1]/max(max(max(orient_score))))
figure(1)
hold off
scatter3(x_score(I),y_score(I),z_score(I),[],orient_score(I))
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
hold on
plot3(0,0,0,'k*','LineWidth',14)


xbins = x_range;
ybins = y_range;
zbins = z_range;
scores = reshape(orient_score,M,M,M);

save('reach_data.mat','M','N','scores','xbins','ybins','zbins');








% %% look at y-z slices
% figure(2)
% x_bnd = .7 + [-.05 .05];
% I = find(x_score >= x_bnd(1) & x_score <= x_bnd(2));
% scatter(y_score(I),z_score(I),[],orient_score(I))
% xlabel('y')
% ylabel('z')
% % xlim([min(y_score) max(y_score)])
% % axis equal
% axis([min(y_score) max(y_score) min(z_score) max(z_score)],'equal')
% 
% %%
% % score = zeros(M,M,M);
% % for i=1:M,
% %   for j=1:M,
% %     for k=1:M,
% %       score(i,j,k) = sum(pos(:,1) >= x_range(i) & pos(:,1) <= x_range(i+1) & pos(:,2) >= y_range(j) & pos(:,2) <= y_range(j+1) & pos(:,3) >= z_range(k) & pos(:,3) <= z_range(k+1));
% %       x_score(i,j,k) = (x_range(i) + x_range(i+1))/2;
% %       y_score(i,j,k) = (y_range(j) + y_range(j+1))/2;
% %       z_score(i,j,k) = (z_range(k) + z_range(k+1))/2;
% %     end
% %   end
% %   i
% % end
% 
% %%
%       drill_dir_constraint = WorldGazeDirConstraint(r,obj.hand_body,obj.drill_axis_on_hand,...
%         obj.drilling_world_axis,obj.default_axis_threshold,[t_vec(end) t_vec(end)]);
% 
% %% run through orient_list looking for things similar enough to a given orientation
% gaze_axis = [1;0;0];
% % quat_palm_for = axis2quat([0;1;0;pi/2]);
% % quat_wrist_for = quatmultiply(axis2quat([0;0;1;pi/2])', quat_palm_for')';
% % quat_wrist_for = rotmat2quat(axis2rotmat([0;0;1;pi/2])*quat2rotmat(quat_palm_for));
% quat_wrist_for = quatTransform([0;-1;0],[1;0;0]);
% quat_palm_for = quatTransform([0;0;1],[1;0;0]);
% % quat_des = quat_wrist_for;
% quat_palm_for_vert_cut = [sqrt(2)/2;0;-sqrt(2)/2;0];
% quat_palm_for_horiz_cut1 = [.5;-.5;-.5;-.5];
% quat_palm_for_horiz_cut2 = [.5;.5;-.5;.5];
% quat_des = quat_palm_for_horiz_cut2;
% qq = zeros(34,1);
% qq(4:6) = quat2rpy(quat_des);
% v.draw(0,qq);
% M = size(orient_list,1);
% orient_bool = false(M,M,M);
% threshold = .1;
% for i=1:M,
%   for j=1:M,
%     for k=1:M,
%       if ~isempty(orient_list{i,j,k})
%         if max((orient_list{i,j,k}*quat_des).^2) > 1 - threshold;
%           orient_bool(i,j,k) = true;
%         end
%       end
% %       for l=1:size(orient_list{i,j,k},1),
% %         if quatDiffAxisInvar(orient_list{i,j,k}(l,:)',quat_des,gaze_axis) > cos(threshold) - 1
% %           orient_bool(i,j,k) = true;
% %           break;
% %         end
% %       end
%     end
%   end
%   i
% end
% I = find(orient_bool);
% figure(2)
% hold off
% scatter3(x_score(I), y_score(I), z_score(I))
% hold on
% plot3(0,0,0,'k*','LineWidth',14)
% xlabel('x')
% ylabel('y')
% zlabel('z')
% axis equal

