classdef ZMPplanner < DrakeSystem
% The input to the system is COM height, foot placement within a preview
% window, and current COM
% The output of the system is the COM trajectory and cost-to-go trajectory
    methods
        function obj = ZMPplanner(window_size,max_contact_pts,dt,g,options)
            % @param window, the time breaks in the preview window
            % @param foot step, the foot step trajectory
            % @height, the COM height in the preview window
            obj = obj@DrakeSystem(0,...
                0,...
                4+window_size*(max_contact_pts*3+1),... % number of inputs are the current planar COM, the position of the contact points, and the com height
                3,...% number of outputs, com positions in the next time step
                true,...
                true);
            obj.window_size = window_size;
            obj.max_contact_pts = max_contact_pts;
            obj.dt = dt;
            obj.A = sparse([1 0 obj.dt 0;0 1 0 obj.dt;0 0 1 0;0 0 0 1]);
            obj.B = sparse([obj.dt^2/2 0;0 obj.dt^2/2;obj.dt 0;0 obj.dt]);
            obj.C = sparse([1 0 0 0;0 1 0 0]);
            obj.g = g;
            % M and N are for dense formulation of optimization
            obj.Mbar = zeros(4*obj.window_size,4); % Mbar = [I;A;A^2;...;A^(n-1)]
            obj.Nbar = zeros(4*obj.window_size,2*obj.window_size); % Nbar = [0 0 ... 0;B 0 .. 0; AB B 0 ... 0;...]
            obj.M = zeros(2*obj.window_size,4); % M = [C;CA;CA^2;...;CA^(N-1)];
            obj.N = zeros(2*obj.window_size);
            obj.M(1:2,:) = obj.C;
            obj.Mbar(1:4,:) = eye(4);
            obj.N(1:2,:) = zeros(2,2*obj.window_size);
            obj.Nbar(1:4,:) = zeros(4,2*obj.window_size);
            for i = 2:obj.window_size
                obj.M((i-1)*2+(1:2),:) = obj.M((i-2)*2+(1:2),:)*obj.A;
                obj.Mbar((i-1)*4+(1:4),:) = obj.Mbar((i-2)*4+(1:4),:)*obj.A;
                obj.N((i-1)*2+(1:2),1:2) = obj.M((i-2)*2+(1:2),:)*obj.B;
                obj.N((i-1)*2+(1:2),3:end) = obj.N((i-2)*2+(1:2),1:end-2);
                obj.Nbar((i-1)*4+(1:4),1:2) = obj.Mbar((i-2)*4+(1:4),:)*obj.B;
                obj.Nbar((i-1)*4+(1:4),3:end) = obj.Nbar((i-2)*4+(1:4),1:end-2);
            end
            if(isfield(options,'supportPolygonConstraints'))
                obj.supportPolygonConstraints = true;
            else
                obj.supportPolygonConstraints = options.supportPolygonConstraints;
            end
        end
        
        function [com_plan,planar_comdot_plan,comddot_plan,zmp_plan,S1,S2] = planning(obj,com0,comdot0,contact_pos,active_contact_flag,z_com,t_breaks)
            tic
            nx = 4;
            nu = 2;
            ny = 2;
            x0 = [com0;comdot0];
            
            z_com_pp = spline(t_breaks,z_com);
            zddot_com = ppval(fnder(z_com_pp,2),t_breaks);
            % The followings are for the dense formulation
            % The decision variables are the weight of the convex support
            % polygon vertices, the com acceleration
            support_vertices = cell(1,obj.window_size);
            inner_support_vertices = cell(1,obj.window_size);
            shrink_factor = 0.8;
            convex_comb = cell(1,obj.window_size);
            support_center = zeros(2,obj.window_size);
            D_val = reshape(repmat(-z_com./(zddot_com+obj.g),2,1),[],1);
            N = obj.N+diag(D_val);
            active_contact_flag = logical(active_contact_flag);
            % This for loop is ugly, try vectorize it later, or use sparse
            % formulation
            for i = 1:obj.window_size
                active_contact_pos = contact_pos(:,active_contact_flag(:,i),i);
                support_vertices_ind_i = convhull(active_contact_pos(1,:),active_contact_pos(2,:));
                support_vertices{i} = active_contact_pos(1:2,support_vertices_ind_i);
                convex_comb{i} = ones(1,length(support_vertices_ind_i));
                support_center(:,i) = mean(active_contact_pos(1:2,:),2);
                inner_support_vertices{i} = shrink_factor*support_vertices{i}+(1-shrink_factor)*repmat(support_center(:,i),1,size(support_vertices{i},2));
            end
            x0Y = obj.M*x0; % This is the effect of initial com on preview com
            x0X = obj.Mbar*x0;
            inner_support_vert_mat = blkdiag(inner_support_vertices{:});
            n_weights = size(inner_support_vert_mat,2);
            convex_comb_mat = blkdiag(convex_comb{:});
            if(obj.supportPolygonConstraints)
            Aeq = [inner_support_vert_mat -N;convex_comb_mat zeros(size(convex_comb_mat,1),2*obj.window_size)];
            beq = [x0Y; ones(obj.window_size,1)];
            lb = [zeros(n_weights,1);-inf(2*obj.window_size,1)];
            ub = [ones(n_weights,1);inf(2*obj.window_size,1)];
            end
            % The cost includes an infinite horizon LQR cost at the end to
            % prevent the acceleration going to infinity
            % construct a LTI system at the end of the preview window
%             ti_end = LinearSystem([],[],obj.A,obj.B,obj.C,-z_com(end)/(zddot_com(end)+obj.g)*eye(2));
%             tilqr_options = struct();
%             tilqr_options.Qy = eye(2);
%             tilqr_options.yd = support_center(:,end);
%             R_end = 1e-2*eye(2);
%             [~,V_end] = tilqr(ti_end,Point(ti_end.getStateFrame,zeros(4,1)),Point(ti_end.getInputFrame,zeros(2,1)),zeros(4),R_end,tilqr_options);
%             if(any(abs(eig(obj.A-obj.B*(R_end\(obj.B'*V_end.S))))>1))
%                 error('The ti system at the end is not stable')
%             end
            V_end.S = zeros(4);
            R = 1e-1*eye(nu);
            R_u = 1e-1*eye(nu*obj.window_size);
            R_u(end-nu+1:end,end-nu+1:end) = zeros(nu);
            Qy = eye(ny);
            Q_zmp = 1*eye(ny*obj.window_size);
            pickUmat = [zeros(nu*obj.window_size,n_weights) eye(nu*obj.window_size)];
            Mbarf = obj.Mbar(end-nx+1:end,:);
            Nbarf = obj.Nbar(end-nx+1:end,:);
            H = pickUmat'*(N'*Q_zmp*N+R_u+Nbarf'*V_end.S*Nbarf)*pickUmat;
            f = ((obj.M*x0-support_center(:))'*Q_zmp*N+...
                (Mbarf*x0)'*V_end.S*Nbarf)*pickUmat;
%             K = -(N'*Q_zmp*N+R_u+Nbarf'*V_end.S*Nbarf)\(obj.M'*Q_zmp*N+Mbarf'*V_end.S*Nbarf)';
%             K = K(1:2,:);
%             if(any(abs(eig(obj.A+obj.B*K))>1))
%                 error('The preview controller is not stable')
%             end
%             [sol,fval,exitflag] = cplexqp((H+H')/2,f,[],[],Aeq,beq,lb,ub,[1/obj.max_contact_pts*ones(n_weights,1);zeros(2*obj.window_size,1)]);
            if(obj.supportPolygonConstraints)
                [sol,fval,exitflag] = cplexqp((H+H')/2,f,[],[],Aeq,beq,lb,ub,[1/obj.max_contact_pts*ones(n_weights,1);zeros(2*obj.window_size,1)]);
            else
                [sol,fval,exitflag] = cplexqp((H+H')/2,f,[],[]);
            end
            if(exitflag<=0)
                error('ZMP planning is not successful')
            end
            % V = x'S1x+2*x'*S2+S3
            S1 = zeros(nx,nx,obj.window_size);
            S2 = zeros(nx,obj.window_size);
%             S3 = zeros(1,obj.window_size);
            for i = obj.window_size-1:-1:1
                Dn = -z_com(i)/(zddot_com(i)+obj.g)*eye(ny,nu);
                P1 = (Dn'*Qy*Dn+R+obj.B'*S1(:,:,i+1)*obj.B);
                P2 = (obj.C'*Qy*Dn+obj.A'*S1(:,:,i+1)*obj.B)';
                P3 = obj.B'*S2(:,i+1)-Dn'*Qy*support_center(:,i);
                S1(:,:,i) = obj.A'*S1(:,:,i+1)*obj.A+obj.C'*Qy*obj.C+(P2'/P1)*P2;
                S2(:,i) = obj.A'*S2(:,i+1)-obj.C'*Qy*support_center(:,i)-(P2'/P1)*P3;
%                 S3(:,i) = S3(:,i+1)+support_center(:,i)'*Qy*support_center(:,i)+(P3'/P1)*P3;
            end
            S1 = SharedDataHandle(S1);
            S2 = SharedDataHandle(S2);
            if(obj.supportPolygonConstraints)
                weights_sol = sol(1:n_weights);
                comddot_preview = sol(n_weights+(1:nu*obj.window_size));
            else
                comddot_preview = sol(1:nu*obj.window_size);
            end
            LIP_state_preview = reshape(obj.Mbar*x0+obj.Nbar*comddot_preview,4,obj.window_size);
            com_plan = [LIP_state_preview(1:2,:);z_com];
            planar_comdot_plan = LIP_state_preview(3:4,:);
            comddot_plan = [reshape(comddot_preview,2,[]);zddot_com];
            zmp_plan = reshape(obj.M*x0+N*comddot_preview,2,obj.window_size);
            toc
        end
        
        
        
        
        function com = output(obj,t,~,u)
            % u = [current planar com, contact planar positions in the preview
            % window, active_contact_flag,com height in the preview window]
%             profile on
%             tic
            nx = 4;
            nu = 2;
            ny = 2;
            x0 = u(1:nx);
            contact_pos = reshape(u(nx+(1:obj.window_size*obj.max_contact_pts*2)),2,obj.max_contact_pts,obj.window_size);
            active_contact_flag = logical(reshape(u(nx+obj.window_size*obj.max_contact_pts*2+...
                (1:obj.window_size*obj.max_contact_pts)),obj.max_contact_pts,obj.window_size));
            z_com = reshape(u(nx+obj.window_size*obj.max_contact_pts*3+(1:obj.window_size)),1,[]);
            t_breaks = t+obj.dt*(0:obj.window_size-1);
            [com_plan,comdot_plan,comddot_plan,zmp_plan,S1,S2] = planning(x0(1:2),x0(3:4),contact_pos,active_contact_flag,z_com,t_breaks);
            com = com_plan(:,2);
           
        end
    end
    properties
        max_contact_pts;
        window_size;
        dt;
        A;
        B;
        C;
        g;
        M;
        N;
        Mbar;
        Nbar;
        supportPolygonConstraints % true if we want the ZMP to lie in the support polygon
    end
end