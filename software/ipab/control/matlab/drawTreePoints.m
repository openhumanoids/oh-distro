function drawTreePoints(points, varargin)
  
  options = struct('frame',false,'lines', false, 'tree', [], 'text', 'Highlighted Points', ...
                  'colour', [0 1 0], 'pointsize', 0.01);
  optionNames = fieldnames(options);
  nArgs = length(varargin);
  if round(nArgs/2)~=nArgs/2
    error('Needs propertyName/propertyValue pairs')
  end  
  for pair = reshape(varargin,2,[])
    inpName = lower(pair{1});    
    if any(strcmp(inpName,optionNames))
      options.(inpName) = pair{2};
    else
      error('%s is not a recognized parameter name',inpName)
    end
  end
  
  lcmClient = LCMGLClient(options.text);
  lcmClient.glColor3f(options.colour(1), options.colour(2), options.colour(3));
  if ~isempty(points)
    if isempty(options.tree)
      assert(size(points, 1) >= 3);
      if size(points, 1) >= 7
        P = points(1:7, :);
      else
        P = points(1:3, :);
      end
    else
      P = options.tree.getVertex(points);
      P = P(1:7, :);
    end

    for p = 1:size(P, 2)
      lcmClient.sphere(P(1:3, p), options.pointsize, 20, 20);
      if options.lines && p > 1
        lcmClient.line3(P(1, p), P(2, p), P(3, p), P(1, p-1), P(2, p-1), P(3, p-1));
      end
      if options.frame
        T = [quat2rotmat(P(4:7, p)) P(1:3, p); [0 0 0 1]];
        applyTransform(lcmClient,T);
        lcmClient.glDrawAxes();
        applyTransform(lcmClient,inv(T));
      end
    end
  end
  lcmClient.switchBuffers();
  
  function applyTransform(lcmClient,T)
    a = rotmat2axis(T(1:3,1:3));
    lcmClient.glTranslated(T(1,4),T(2,4),T(3,4));
    lcmClient.glRotated(a(4)*180/pi,a(1),a(2),a(3));
  end
end