classdef ZMPplanner < DrakeSystem
% The input to the system is COM height, foot placement within a preview
% window, and current COM
% The output of the system is the COM trajectory and cost-to-go trajectory
    methods
        function obj = ZMPplanner(window_size,max_contact_pts,dt,g)
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
        end
        
        function [com_plan,planar_comdot_plan,comddot_plan,zmp_plan,S1,S2] = planning(obj,com0,comdot0,contact_pos,active_contact_flag,z_com,t_breaks,options)
%             profile on
%             tic
            if(~isfield(options,'shrink_factor'))
                options.shrink_factor = 0.8;
            end
            if(~isfield(options,'useQP'))
                options.useQP = true;
            end
            if(~isfield(options,'supportPolygonConstraints'))
                options.supportPolygonConstraints = true;
            end
            nx = 4;
            nu = 2;
            ny = 2;
            x0 = [com0;comdot0];
            
            preview_size = length(t_breaks);
            if(preview_size>obj.window_size)
                error('t_breaks has %d knots, but zmp planner accepts %d at most',preview_size,obj.window_size);
            end
            if(preview_size == obj.window_size)
                M = obj.M;
                N = obj.N;
                Mbar = obj.Mbar;
                Nbar = obj.Nbar;
            else % If the time window is shorter than the presumed window
                M = obj.M(1:ny*preview_size,:);
                N = obj.N(1:ny*preview_size,1:nu*preview_size);
                Mbar = obj.Mbar(1:nx*preview_size,:);
                Nbar = obj.Nbar(1:nx*preview_size,1:nu*preview_size);
            end
            % deal with the case that the final time step is not as big as
            % dt
            dt_end = t_breaks(end)-t_breaks(end-1);
            if(dt_end ~=obj.dt)
                A_end = [eye(2) dt_end*eye(2);zeros(2) eye(2)];
                B_end = [dt_end^2/2*eye(2);dt_end*eye(2)];
                Mbar(end-nx+1:end,:) = A_end*Mbar(end-2*nx+1:end-nx,:);
                M(end-ny+1:end,:) = obj.C*Mbar(end-nx+1:end,:);
            end
            z_com_pp = spline(t_breaks,z_com);
            zddot_com = ppval(fnder(z_com_pp,2),t_breaks);
            % The followings are for the dense formulation
            % The decision variables are the weight of the convex support
            % polygon vertices, the com acceleration
            support_vertices = cell(1,preview_size);
            inner_support_vertices = cell(1,preview_size);
            convex_comb = cell(1,preview_size);
            support_center = zeros(2,preview_size);
            D_val = reshape(repmat(-z_com./(zddot_com+obj.g),2,1),[],1);
            N = N+diag(D_val);
            active_contact_flag = logical(active_contact_flag);
            num_support_vertices = 0;
            support_vertices_row = zeros(obj.max_contact_pts*preview_size*2,1);
            support_vertices_col = zeros(obj.max_contact_pts*preview_size*2,1);
            inner_support_vertices_val = zeros(obj.max_contact_pts*preview_size*2,1);
            support_vertices_ind = 0;
            convex_comb_row = zeros(obj.max_contact_pts*preview_size,1);
            convex_comb_col = (1:obj.max_contact_pts*preview_size)';
            convex_comb_val = ones(obj.max_contact_pts*preview_size,1);
            for i = 1:preview_size
                active_contact_pos = contact_pos(:,active_contact_flag(:,i),i);
                support_center(:,i) = mean(active_contact_pos(1:2,:),2);
                support_vertices_ind_i = convhull(active_contact_pos(1,:),active_contact_pos(2,:));
                num_support_vertices_ind_i = length(support_vertices_ind_i);
                support_vertices_row(support_vertices_ind+(1:num_support_vertices_ind_i*2)) =...
                    reshape([ones(1,num_support_vertices_ind_i);2*ones(1,num_support_vertices_ind_i)],[],1)+(i-1)*2;
                support_vertices_col(support_vertices_ind+(1:num_support_vertices_ind_i*2)) = ...
                    reshape(bsxfun(@plus,num_support_vertices+(1:num_support_vertices_ind_i),[0;0]),[],1);
                inner_support_vertices_val(support_vertices_ind+(1:num_support_vertices_ind_i*2)) = ...
                    reshape(bsxfun(@plus,options.shrink_factor*active_contact_pos(:,support_vertices_ind_i),(1-options.shrink_factor)*support_center(:,i)),[],1);
                convex_comb_row(num_support_vertices+(1:num_support_vertices_ind_i)) = i*ones(num_support_vertices_ind_i,1);
                num_support_vertices = num_support_vertices+num_support_vertices_ind_i;
                support_vertices_ind = support_vertices_ind+2*num_support_vertices_ind_i;
%                 support_vertices{i} = active_contact_pos(1:2,support_vertices_ind_i);
%                 convex_comb{i} = ones(1,length(support_vertices_ind_i));
                
            end
            x0Y = M*x0; % This is the effect of initial com on preview com
            x0X = Mbar*x0;
            inner_support_vert_mat = sparse(support_vertices_row(1:support_vertices_ind),...
                support_vertices_col(1:support_vertices_ind),inner_support_vertices_val(1:support_vertices_ind),...
                2*preview_size,num_support_vertices);
            n_weights = num_support_vertices;
%             convex_comb_mat = blkdiag(convex_comb{:});
            convex_comb_mat = sparse(convex_comb_row(1:num_support_vertices),...
                convex_comb_col(1:num_support_vertices),convex_comb_val(1:num_support_vertices),...
                preview_size,num_support_vertices);
            
            R = 1e-1*eye(nu);
            Qy = eye(ny);

            Dn = -z_com(end)/(zddot_com(end)+obj.g)*eye(ny,nu);
            V_end_S1 = dare(full(obj.A),full(obj.B),full(obj.C'*Qy*obj.C),R+Dn'*Qy*Dn,full(obj.C'*Qy*Dn),eye(nx));
            P1 = (Dn'*Qy*Dn+R+obj.B'*V_end_S1*obj.B);
            P2 = (obj.C'*Qy*Dn+obj.A'*V_end_S1*obj.B)';
            V_end_S2 = (eye(nx)-obj.A'+(P2'/P1)*obj.B')\(-obj.C'*Qy*support_center(:,end)+(P2'/P1)*Dn'*Qy*support_center(:,end));
                
            if(options.useQP)
                if(options.supportPolygonConstraints)
                Aeq = [inner_support_vert_mat -N;convex_comb_mat sparse(preview_size,nu*preview_size)];
                beq = [x0Y; ones(preview_size,1)];
                lb = [zeros(n_weights,1);-inf(nu*preview_size,1)];
                ub = [ones(n_weights,1);inf(nu*preview_size,1)];
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
                R_u = 1e-1*eye(nu*preview_size);
                Q_zmp = 0*eye(ny*preview_size);
                pickUmat = [sparse(nu*preview_size,n_weights) speye(nu*preview_size)];
                Mbarf = Mbar(end-nx+1:end,:);
                Nbarf = Nbar(end-nx+1:end,:);
                H = pickUmat'*(N'*Q_zmp*N+R_u+Nbarf'*V_end_S1*Nbarf)*pickUmat;
                f = ((M*x0-support_center(:))'*Q_zmp*N+...
                    (Mbarf*x0)'*V_end_S1*Nbarf+V_end_S2'*Nbarf)*pickUmat;
    %             K = -(N'*Q_zmp*N+R_u+Nbarf'*V_end.S*Nbarf)\(obj.M'*Q_zmp*N+Mbarf'*V_end.S*Nbarf)';
    %             K = K(1:2,:);
    %             if(any(abs(eig(obj.A+obj.B*K))>1))
    %                 error('The preview controller is not stable')
    %             end
    %             [sol,fval,exitflag] = cplexqp((H+H')/2,f,[],[],Aeq,beq,lb,ub,[1/obj.max_contact_pts*ones(n_weights,1);zeros(2*obj.window_size,1)]);
                if(options.supportPolygonConstraints)
                    model = struct();
                    model.A = Aeq;
                    model.obj = 2*f';
                    model.sense = '=';
                    model.rhs = beq;
                    model.lb = lb;
                    model.ub = ub;
                    model.Q = sparse((H+H')/2);
                    params = struct();
                    params.outputflag = 0;
                    results = gurobi(model,params);
                else
    %                 [sol,fval,exitflag] = cplexqp((H+H')/2,f,[],[]);
                    model = struct();
                    model.obj = 2*f;
                    model.A = sparse(1,length(f));
                    model.sense = '=';
                    model.rhs = 0;
                    model.Q = sparse((H+H')/2);
                    params = struct();
                    params.outputflag = 0;
                    results = gurobi(model,params);
                end
                if(~strcmp(results.status, 'OPTIMAL'))
                    error('ZMP planning is not successful');
                end
            else % use LP instead
                % For LP formulation, add a slack variable to penalize the
                % L_infinity norm of u. The TILQR cost cannot be added at
                % the end since that incurs quadratic cost
                Aeq = [inner_support_vert_mat -N sparse(2*preview_size,2);convex_comb_mat sparse(preview_size,preview_size*nu+2)];
                Aineq = [inner_support_vert_mat sparse(2*preview_size,nu*preview_size) -spones(ones(2*preview_size,1)) sparse(2*preview_size,1);...
                    inner_support_vert_mat sparse(2*preview_size,nu*preview_size) spones(ones(2*preview_size,1)) sparse(2*preview_size,1);...
                    sparse(nu*preview_size,n_weights) speye(nu*preview_size) sparse(nu*preview_size,1) -spones(ones(nu*preview_size,1));...
                    sparse(nu*preview_size,n_weights) speye(nu*preview_size) sparse(nu*preview_size,1) spones(ones(nu*preview_size,1))];
                model = struct();
                model.A = [Aeq;Aineq];
%                 model.rhs = [x0Y;ones(preview_size,1);support_center(:);support_center(:)];
                model.rhs = [x0Y;ones(preview_size,1);support_center(:);support_center(:);zeros(2*nu*preview_size,1)];
                model.lb = [zeros(n_weights,1);-inf(nu*(preview_size-1),1);-inf;-inf;0;0];
                model.ub = [ones(n_weights,1);inf(nu*(preview_size-1),1);inf;inf;inf;inf];
                model.obj = [zeros(n_weights+nu*preview_size,1);1;1];
                model.sense = strcat(repmat('=',1,3*preview_size),repmat('<',1,2*preview_size),repmat('>',1,2*preview_size),repmat('<',1,nu*preview_size),repmat('>',1,nu*preview_size));
                params = struct();
                params.outputflag = 0;
                results = gurobi(model,params);
                if(~strcmp(results.status, 'OPTIMAL'))
                    error('ZMP planning is not successful');
                end
            end
            % V = x'S1x+2*x'*S2+S3
            S1 = zeros(nx,nx,preview_size);
            S2 = zeros(nx,preview_size);
            S1(:,:,end) = V_end_S1;
            S2(:,end) = V_end_S2;
%             S3 = zeros(1,obj.window_size);
            for i = preview_size-1:-1:1
                Dn = -z_com(i)/(zddot_com(i)+obj.g)*eye(ny,nu);
                P1 = (Dn'*Qy*Dn+R+obj.B'*S1(:,:,i+1)*obj.B);
                P2 = (obj.C'*Qy*Dn+obj.A'*S1(:,:,i+1)*obj.B)';
                P3 = obj.B'*S2(:,i+1)-Dn'*Qy*support_center(:,i);
                S1(:,:,i) = obj.A'*S1(:,:,i+1)*obj.A+obj.C'*Qy*obj.C+(P2'/P1)*P2;
                S2(:,i) = obj.A'*S2(:,i+1)-obj.C'*Qy*support_center(:,i)-(P2'/P1)*P3;
%                 S3(:,i) = S3(:,i+1)+support_center(:,i)'*Qy*support_center(:,i)+(P3'/P1)*P3;
            end
            if(options.supportPolygonConstraints)
%                 weights_sol = sol(1:n_weights);
%                 comddot_preview1 = sol(n_weights+(1:nu*preview_size));
                weights_sol = results.x(1:n_weights);
                comddot_preview = results.x(n_weights+(1:nu*preview_size));
            else
%                 comddot_preview = sol(1:nu*preview_size);
                comddot_preview = results.x((1:nu*preview_size));
            end
            LIP_state_preview = reshape(Mbar*x0+Nbar*comddot_preview,nx,preview_size);
            com_plan = [LIP_state_preview(1:2,:);z_com];
            planar_comdot_plan = LIP_state_preview(3:4,:);
            comddot_plan = [reshape(comddot_preview,2,[]);zddot_com];
            zmp_plan = reshape(M*x0+N*comddot_preview,ny,preview_size);
%             toc
%             profile off
%             profile viewer
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
            options = struct();
            [com_plan,comdot_plan,comddot_plan,zmp_plan,S1,S2] = planning(x0(1:2),x0(3:4),contact_pos,active_contact_flag,z_com,t_breaks,options);
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
    end
end