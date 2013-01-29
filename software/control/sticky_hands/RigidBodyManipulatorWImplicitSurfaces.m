classdef RigidBodyManipulatorWImplicitSurfaces < RigidBodyManipulator
  
  properties
    params
  end
  
  methods
    function obj = RigidBodyManipulatorWImplicitSurfaces(varargin)
      obj = obj@RigidBodyManipulator(varargin{:});
    end
    
    function obj = setManipulandParams(obj,params)
      obj.params = params;
    end
    
    function [pos,vel,normal,mu] = collisionDetect(obj,contact_pos)
      % for each column of contact_pos, find the closest point in the world
      % geometry, and it's (absolute) position and velocity, (unit) surface
      % normal, and coefficent of friction.
      
      % for now, just implement a ground height at y=0
      n = size(contact_pos,2);
      pos = [contact_pos(1:2,:);zeros(1,n)+1];
      normal = [zeros(2,n); ones(1,n)];
      
      % find the closest point on superellipsoid and its normal.
      
      %[pos,normal]=closestPointsOnZAlignedSuperEllipsoid(contact_pos,params);
      [pos,normal]=closestPointsOnZAlignedSuperEllipsoid_NumericalNormals(obj,contact_pos);
      
      
      %[pos,normal] =closestPointsOnSphere(contact_pos);
      %       tic
      %[pos,normal] =closestPointsOnSphere2(contact_pos);
      %       toc
      % Ellipsoid is 25 times slower as there is no closed form expression
      % and you have to do a root search
      % SuperEllipsoid and Torus is 500 times slower
      
      vel = zeros(3,n); % static world assumption (for now)
      mu = ones(1,n);
    end
    
    function [ps,normals]=closestPointsOnZAlignedSuperEllipsoid_NumericalNormals(obj,p)
     
      params = obj.params;
      xc =params.xc;
      a = params.a;
      epsilon = params.epsilon;
      n=size(p,2);
      for ind =1:n,
        q = p(:,ind)-xc(:);
        qs=approxClosestPoint_zsuperellipsoid(obj,q);
        ps(:,ind) = qs+xc(:);
        
        px = qs(1); py = qs(2); pz = qs(3);
        e_rat =  epsilon(1)/epsilon(2);
        temp = e_rat*(abs(px/a(1)).^(2/epsilon(1))+abs(py/a(2)).^(2/epsilon(1))).^(e_rat-1);
        dPhix =temp.*((2/epsilon(1))*(abs(px/a(1))).^((2/epsilon(1))-1)).*sign(px/a(1))*(1/a(1));
        dPhiy =temp.*((2/epsilon(1))*(abs(py/a(2))).^((2/epsilon(1))-1)).*sign(py/a(2))*(1/a(2));
        temp2 = e_rat*(abs(pz/a(3)).^(2/epsilon(1))).^(e_rat-1);
        dPhiz =temp2.*((2/epsilon(1))*(abs(pz/a(3))).^((2/epsilon(1))-1)).*sign(pz/a(3))*(1/a(3));
        nnorm = (dPhix.^2+dPhiy.^2+dPhiz.^2).^0.5;
        
        n(1) =dPhix./nnorm;
        n(2) =dPhiy./nnorm;
        n(3) =dPhiz./nnorm;
        n=n(:);
        
        [~,ni] = geval(@(q)closestpointfun_superellipsoid(obj,q),q,struct('grad_method','numerical'));
        normals(:,ind)=ni(:);
        
        %          if(sum(n(:)-ni(:))>1e-6)
        %             nanalytical = n'
        %             ni
        %          end
        
      end
    end
    function [phi] = closestpointfun_superellipsoid(obj,p)

      params = obj.params;
      xc =params.xc;
      a = params.a;
      epsilon = params.epsilon;
      q=approxClosestPoint_zsuperellipsoid(obj,p);
      %   p'
      %   q'
      px = q(1); py = q(2); pz = q(3);
      %px = p(1); py = p(2); pz = p(3);
      e_rat =  epsilon(1)/epsilon(2);
      temp = e_rat*(abs(px/a(1)).^(2/epsilon(1))+abs(py/a(2)).^(2/epsilon(1))).^(e_rat-1);
      dPhix =temp.*((2/epsilon(1))*(abs(px/a(1))).^((2/epsilon(1))-1)).*sign(px/a(1))*(1/a(1));
      dPhiy =temp.*((2/epsilon(1))*(abs(py/a(2))).^((2/epsilon(1))-1)).*sign(py/a(2))*(1/a(2));
      temp2 = e_rat*(abs(pz/a(3)).^(2/epsilon(1))).^(e_rat-1);
      dPhiz =temp2.*((2/epsilon(1))*(abs(pz/a(3))).^((2/epsilon(1))-1)).*sign(pz/a(3))*(1/a(3));
      nnorm = (dPhix.^2+dPhiy.^2+dPhiz.^2).^0.5;
      
      n(1) =dPhix./nnorm;
      n(2) =dPhiy./nnorm;
      n(3) =dPhiz./nnorm;
      n=n(:);
      
      relpos = p - q;
      s = sign(sum(relpos.*n,1)); % using analytical n as approximation for calculating real normals will this work?
      phi = (sqrt(sum(relpos.^2,1)).*s)';
      
    end
    function [q]=approxClosestPoint_zsuperellipsoid(obj,p)
      params = obj.params;
      epsilon= params.epsilon;
      a = params.a;
      q = p(:);
      if(epsilon(2)==epsilon(1))&&(epsilon(2)~=1) %if box
        % if outside box.
        inside=(-a(1)<=q(1))&(q(1)<=a(1))& (-a(2)<=q(2))&(q(2)<=a(2))& (-a(3)<=q(3))&(q(3)<=a(3));
        if(~inside)
          %disp('~inside')
          for j= 1:3,
            vert=p(j);
            b = [-a(j) a(j)];
            vert = max(vert,min(b));
            vert = min(vert,max(b));
            q(j) =vert;
          end
        else
          %disp('inside')
          b = [-a(:);  a(:)];
          ind= [1:3,1:3]';
          t = [q(:); q(:)];
          [~,i] = min(abs(t(:)-b(:)));
          q(ind(i))=b(i);
        end
        
      elseif (a(1)==a(2))&&(epsilon(2)~=epsilon(1))&&(epsilon(2)~=1) %if cylinder
        r=a(2);
        rc = sum(q(1:2).^2).^0.5;
        azimuth = atan2(q(2),q(1));
        xyinside = (rc<=r);
        zinside = (-a(3)<=q(3))&(q(3)<=a(3));
        % two options xyclamp or z clamp
        qxy(1) = r*cos(azimuth);
        qxy(2) = r*sin(azimuth);
        qxy(3) = q(3);
        qz=q(:);
        b = [-a(3);  a(3)];
        [~,i] = min(abs(q(3)-b(:)));
        qz(3) = b(i); % clamp in z
        
        if(xyinside)&&(zinside)
          %disp('(xyinside)&&(zinside)')
          d=[sum((qxy(:)-p(:)).^2).^0.5;sum((qz(:)-p(:)).^2).^0.5];
          if(d(1)<d(2))
            q=qxy(:);
          else
            q=qz(:);
          end
          
        elseif(xyinside)&&(~zinside)
          %disp('clamp in z')
          q=qz(:); % clamp in z
        elseif(~xyinside)&&(zinside)
          %disp('clamp in xy')
          q=qxy(:); % clamp in xy
        elseif(~xyinside)&&(~zinside)
          %disp('clamp in xy and z ');
          q(1:2) = qxy(1:2); % clamp in xy
          q(3) = qz(3); % clamp in z
          q = q(:);
        end
      elseif  (epsilon(1)==epsilon(2))&&(epsilon(2)==1) %if sphere or axis aligned ellipsoid
        qs = q./a(:);
        q = (qs/norm(qs)).*a(:);
      end
      
    end
    
    
    function [ps,normals]=closestPointsOnZAlignedSuperEllipsoid(obj,p)
      % p is input query points.
      % p must be dim x n
      % returns closest pts on sphere and the corresponding normals
      params = obj.params;
      xc =params.xc;
      a = params.a;
      epsilon = params.epsilon;
      n=size(p,2);
      for ind =1:n,
        q = p(:,ind)-xc(:); % convert to object frame with the origin at the geomertic center
        %====BOX
        if(epsilon(2)==epsilon(1))&&(epsilon(2)~=1) %if box
          % if outside box.
          inside=(-a(1)<=q(1))&(q(1)<=a(1))& (-a(2)<=q(2))&(q(2)<=a(2))& (-a(3)<=q(3))&(q(3)<=a(3));
          if(~inside)
            for j= 1:3,
              vert=q(j);
              b = [-a(j) a(j)];
              vert = max(vert,min(b));
              vert = min(vert,max(b));
              q(j) =vert;
            end
          else
            b = [-a(:);  a(:)];
            k= [1:3,1:3]';
            t = [q(:); q(:)];
            [~,j] = min(abs(t(:)-b(:)));
            q(k(j))=b(j);
          end
          %====CYLINDER
        elseif (a(1)==a(2))&&(epsilon(2)~=epsilon(1))&&(epsilon(2)~=1) %if cylinder
          r=a(2);
          rc = sum(q(1:2).^2).^0.5;
          azimuth = atan2(q(2),q(1));
          xyinside = (rc<=r);
          zinside = (-a(3)<=q(3))&(q(3)<=a(3));
          % two options xyclamp or z clamp
          qxy(1) = r*cos(azimuth);
          qxy(2) = r*sin(azimuth);
          qxy(3) = q(3);
          qz=q(:);
          b = [-a(3);  a(3)];
          [~,i] = min(abs(q(3)-b(:)));
          qz(3) = b(i); % clamp in z
          
          if(xyinside)&&(zinside)
            d=[sum((qxy(:)-q(:)).^2).^0.5;sum((qz(:)-q(:)).^2).^0.5];
            if(d(1)<d(2))
              q=qxy(:);
            else
              q=qz(:);
            end
          elseif(xyinside)&&(~zinside)
            q=qz(:); % clamp in z
          elseif(~xyinside)&&(zinside)
            q=qxy(:); % clamp in xy
          elseif(~xyinside)&&(~zinside)
            q(1:2) = qxy(1:2); % clamp in xy
            q(3) = qz(3); % clamp in z
            q = q(:);
          end
          %====SPHERE
        elseif  (epsilon(1)==epsilon(2))&&(epsilon(2)==1) %if sphere or axis aligned ellipsoid
          qs = q./a(:);
          q = (qs/norm(qs)).*a(:);
        end
        ps(:,ind) = q+xc(:); % transform back to world frame
        %
        % get normals
        x = q(1); y = q(2); z = q(3);
        e_rat =  epsilon(1)/epsilon(2);
        temp = e_rat*(abs(x/a(1)).^(2/epsilon(1))+abs(y/a(2)).^(2/epsilon(1))).^(e_rat-1);
        dPhix =temp.*((2/epsilon(1))*(abs(x/a(1))).^((2/epsilon(1))-1)).*sign(x/a(1))*(1/a(1));
        dPhiy =temp.*((2/epsilon(1))*(abs(y/a(2))).^((2/epsilon(1))-1)).*sign(y/a(2))*(1/a(2));
        temp2 = e_rat*(abs(z/a(3)).^(2/epsilon(1))).^(e_rat-1);
        dPhiz =temp2.*((2/epsilon(1))*(abs(z/a(3))).^((2/epsilon(1))-1)).*sign(z/a(3))*(1/a(3));
        nnorm = (dPhix.^2+dPhiy.^2+dPhiz.^2).^0.5;
        nx=dPhix./nnorm;
        ny=dPhiy./nnorm;
        nz=dPhiz./nnorm;
        
        
        normals(:,ind)=[nx;ny;nz];
      end
    end
    
    
    function F = myObjFun(obj,z)
      params = obj.params;
      p = params.p;
      x = evalSuperEllipsoidPoint(z,params);
      F = ((x(1)-p(1))^2+(x(2)-p(2))^2+(x(3)-p(3))^2);
    end
    
    function x = evalSuperEllipsoidPoint(obj,z)
      params = obj.params;
      epsilon=params.epsilon;
      a=params.a;
      eta=z(1); % latitude - along the main dimension.
      w=z(2); % longitude
      %   etamax=pi/2;  etamin=-pi/2;
      %   wmax=pi;  wmin=-pi;
      
      % super ellipsoid x-axis form.
      % x(1) = a(1)* sign(sin(eta))* abs(sin(eta))^epsilon(2);
      % x(2) = a(2)* abs(cos(eta))^epsilon(2)* sign(sin(w))* abs(sin(w))^epsilon(1);
      %  x(3) = a(3)* abs(cos(eta))^epsilon(2)* sign(cos(w))* abs(cos(w))^epsilon(1);
      
      % super ellipsoid y-axis form.
      x(1) = a(1)* abs(cos(eta))^epsilon(2)* sign(sin(w))* abs(sin(w))^epsilon(1);
      x(2) = a(2)* sign(sin(eta))* abs(sin(eta))^epsilon(2);
      x(3) = a(3)* abs(cos(eta))^epsilon(2)* sign(cos(w))* abs(cos(w))^epsilon(1);
      
      % super ellipsoid z-axis form.
      %   x(1) = a(1)* abs(cos(eta))^epsilon(2)* sign(cos(w))* abs(cos(w))^epsilon(1);
      %   x(2) = a(2)* abs(cos(eta))^epsilon(2)* sign(sin(w))* abs(sin(w))^epsilon(1);
      %   x(3) = a(3)* sign(sin(eta))* abs(sin(eta))^epsilon(2);
    end
    
    
    function [ps,normals,y,dndx]=closestPointsOnSphere(p)
      % p is input query points.
      % p must be dim x n
      % returns closest pts on sphere and the corresponding normals
      
      xc = [0 0 0];
      R =  0.05;
      
      dim = size(p,1);
      nz =size(p,2);
      y = p-repmat(xc(:),1,nz); % defined in the sphere coordinate frame.
      norm_y = repmat(sum(y.^2,1).^0.5,dim,1); %y/norm(y)
      [~,col] = find(norm_y==0); % p is equal to xc ==> all points on the sphere are equidistant.
      norm_y(:,col)=1;
      ps= repmat(xc(:),1,nz)+R*(y./norm_y);
      if(~isempty(col))
        ps(1,unique(col))= ps(1,unique(col)) + R;
      end
      %n = [2*(ps-repmat(xc(:),1,nz))];
      % get unit normals and its derivative in cartesian space.
      
      dndx = zeros(dim*nz,dim);
      for i=1:nz,
        ni = 2.0*(p(:,i)-xc(:));
        dni=2.0*eye(dim);
        
        ni = 2.0*(ps(:,i)-xc(:)); % dpsdx does not seem to have much effect
        [~,dy] = normalize_v_dv(y(:,i),eye(dim));
        dpsdx = R*dy;
        dni=2.0*dpsdx;
        
        [ni,dni] = normalize_v_dv(ni,dni);
        normals(:,i)=ni;
        dndx(dim*(i-1)+1:dim*i,:)=dni;
      end
    end
  end
  
  methods (Static=true)  
    function [ps,normals,y,dndx]=closestPointsOnSphere2(p)
      % p is input query points.
      % p must be dim x n
      % returns closest pts on sphere and the corresponding normals
      
      xc =[0;0;0];
      R =  0.05;
      
      dim = size(p,1);
      nz =size(p,2);
      y = p-repmat(xc(:),1,nz); % defined in the sphere coordinate frame.
      
      %n = [2*(ps-repmat(xc(:),1,nz))];
      % get unit normals and its derivative in cartesian space.
      dndx = zeros(dim*nz,dim);
      for i=1:nz,
        d_mean = 1;
        p_norm = norm(y(:,i));
        lambda0  = (sqrt(d_mean*p_norm^2)-1)/d_mean;
        options = optimset('Display','off');
        lambda = fzero(@(x) myfun2(x,y(:,i)),lambda0,options);
        ps(:,i)  = xc(:)+R*((eye(3)+lambda*eye(3))\y(:,i));
        
        
        ni = 2.0*(p(:,i)-xc(:));
        dni=2.0*eye(dim);
        
        
        ni = 2.0*(ps(:,i)-xc(:));
        [~,dy] = normalize_v_dv(y(:,i),eye(dim));
        dpsdx = R*dy;
        dni=2.0*dpsdx;
        
        [ni,dni] = normalize_v_dv(ni,dni);
        normals(:,i)=ni;
        dndx(dim*(i-1)+1:dim*i,:)=dni;
      end
    end
    
    function val = epsabs(x)
      es=1e-10;
      val = ((x.^2+es^2).^0.5);
    end
    function val = epssign(x)
      es=1e-10;
      val = x./((x.^2+es^2).^0.5);
    end
    function val = epsdsign(x)
      es=1e-10;
      val = es^2./((x.^2+es^2).^1.5);
    end
  end
  
end