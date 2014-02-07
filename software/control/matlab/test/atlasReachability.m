function atlasReachability

% a visualizer process for visualizing atlas state and relevant variables

% load robot model
r = Atlas();
r = removeCollisionGroupsExcept(r,{});
r = compile(r);
r.setInitialState(zeros(getNumStates(r),1));
v = r.constructVisualizer;

utorso_ind = findLinkInd(r,'utorso');
pelvis_ind = findLinkInd(r,'pelvis');
rhand_ind = findLinkInd(r,'r_hand');

x = r.getInitialState();
x(3) = 0.6;
q = x(1:getNumDOF(r));

kinsol = doKinematics(r,q);
utorso_pos = forwardKin(r,kinsol,utorso_ind,[0;0;0]);
pelvis_pos = forwardKin(r,kinsol,pelvis_ind,[0;0;0]);
rhand_pos = forwardKin(r,kinsol,rhand_ind,[0;-0.125;0]);

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'reachability');


[jlmin,jlmax] = getJointLimits(r);

r_arm_usy = findJointIndices(r,'r_arm_usy');
r_arm_shx = findJointIndices(r,'r_arm_shx');
r_arm_elx = findJointIndices(r,'r_arm_elx');
r_arm_ely = findJointIndices(r,'r_arm_ely');
r_arm_uwy = findJointIndices(r,'r_arm_uwy');
r_arm_mwx = findJointIndices(r,'r_arm_mwx');


N = 7;
r_arm_usy_angles = linspace(jlmin(r_arm_usy),jlmax(r_arm_usy),N);
r_arm_shx_angles = linspace(jlmin(r_arm_shx),jlmax(r_arm_shx),N);
r_arm_elx_angles = linspace(jlmin(r_arm_elx),jlmax(r_arm_elx),N);
r_arm_ely_angles = linspace(jlmin(r_arm_ely),jlmax(r_arm_ely),N);
r_arm_uwy_angles = linspace(jlmin(r_arm_uwy),jlmax(r_arm_uwy),N);
r_arm_mwx_angles = linspace(jlmin(r_arm_mwx),jlmax(r_arm_mwx),N);

X = zeros(7,N^6);

% GROSS, could do this with base-6 counting
tic;
i=1;
for usy=1:N
	q(r_arm_usy) = r_arm_usy_angles(usy);
	for shx=1:N
		q(r_arm_shx) = r_arm_shx_angles(shx);
		for elx=1:N
			q(r_arm_elx) = r_arm_elx_angles(elx);
			for ely=1:N
				q(r_arm_ely) = r_arm_ely_angles(ely);
				for uwy=1:N
					q(r_arm_uwy) = r_arm_uwy_angles(uwy);
					for mwx=1:N
						q(r_arm_mwx) = r_arm_mwx_angles(mwx);
						kinsol = doKinematics(r,q);
						X(:,i) = forwardKin(r,kinsol,rhand_ind,[0;-0.125;0],2);
						i=i+1;
					end
				end
			end
		end
	end
end
toc

% bin reachable workspace, count number of different orientations in each
M = 50; % bins
xbins = linspace(min(X(1,:)),max(X(1,:)+1e-8),M+1);
ybins = linspace(min(X(2,:)),max(X(2,:)+1e-8),M+1);
zbins = linspace(min(X(3,:)),max(X(3,:)+1e-8),M+1);

maxscore=30;
red=PPTrajectory(foh(linspace(0,maxscore,5),[1 .75 .5 .25 0])); 
green=PPTrajectory(foh(linspace(0,maxscore,5),[0 .25 .5 .95 1])); 

% summ=0;
count=1;
scores = zeros(M,M,M);
for xbin=2:M+1
	xbin
	for ybin=2:M+1
		for zbin=2:M+1
			inbin = sum(X(1:3,:)>=repmat([xbins(xbin-1);ybins(ybin-1);zbins(zbin-1)],1,N^6) & X(1:3,:)<repmat([xbins(xbin);ybins(ybin);zbins(zbin)],1,N^6),1)==3;
			score = numUniqueQuats(X(4:7,inbin));
			scores(xbin-1,ybin-1,zbin-1) = score;
			if score>0
				bin_center = [(xbins(xbin)-xbins(xbin-1))/2 + xbins(xbin-1); 
											(ybins(ybin)-ybins(ybin-1))/2 + ybins(ybin-1); 
											(zbins(zbin)-zbins(zbin-1))/2 + zbins(zbin-1)];
				lcmgl.glColor4f(red.eval(score),green.eval(score),0,0.5);
				lcmgl.sphere(bin_center, 0.02, 20, 20);
% 				summ=summ+score;
			end
			count  = count+1;
		end
	end
end
% summ
count 

save('reach_data.mat','M','N','scores','xbins','ybins','zbins');
v.draw(0,x);


% lcmgl.glColor4f(1, 0, 0, 0.2);
% for i=1:N^6
% 	lcmgl.sphere(X(:,i), 0.01, 10, 10);
% end

% lcmgl.glColor4f(0,0,1,0.2);
% lcmgl.glPushMatrix();
% lcmgl.glTranslated(utorso_pos(1),utorso_pos(2),utorso_pos(3));
% lcmgl.sphere([0.55;-0.3;0], 0.3, 20, 20);
% lcmgl.sphere([0.05;-0.5;-0.5], 0.3, 20, 20);
% lcmgl.glPopMatrix();


% load reach_data.mat 
% lcmgl.glColor4f(1, 0, 0, 0.2);
% lcmgl.glColor4f(0,0,1,0.2);
% lcmgl.glPushMatrix();
% lcmgl.glTranslated(pelvis_pos(1),pelvis_pos(2),pelvis_pos(3));
% for i=1:length(x_score)
% 	lcmgl.sphere([x_score(i);y_score(i);z_score(i)], 0.01, 10, 10);
% end
% lcmgl.glPopMatrix();


% 
% figure(1)
% hold off
% scatter3(x_score(:),y_score(:),z_score(:),[],orient_score(:))
% xlabel('x')
% ylabel('y')
% zlabel('z')
% axis equal
% hold on
% plot3(0,0,0,'k*','LineWidth',14)

% lcmgl.glColor4f(0,1,0,0.2);
% lcmgl.glPushMatrix();
% lcmgl.glTranslated(utorso_pos(1),utorso_pos(2),utorso_pos(3));
% lcmgl.sphere([0.1;-0.35;0.3], 0.75, 100, 100);
% lcmgl.glPopMatrix();

lcmgl.switchBuffers();


	function n=numUniqueQuats(quats)
		nquats=size(quats,2);
		uniq = ones(nquats,1);
		for ii=1:nquats
			for jj=ii+1:nquats
				if uniq(ii) && uniq(jj) && abs(quatDist(quats(:,ii),quats(:,jj)))<0.1
					uniq(jj) = 0;
				end
			end	
		end
		n=sum(uniq);
	end

	function d=quatDist(q1,q2)
		aa = quat2axis(quatDiff(q1,q2));
		d=aa(4);
	end

end