function test()
    addpath('../pod-build/lib');
    addpath('../matlab/bailey');

    clear functions;  % this clears all mex files from memory (and deletes all old objects)
    
    ptr = cam_initialize();
    
    fprintf('running...\n');
        
    R = eye(6) * 0.05^2;  % system noise
    Q = [ 1 1 1 ] * 0.01^2;  % observation covariance; code assumes this is a diagonal matrix for conveience
    
    while ( true )
        success = cam_waitForObservations(ptr, 5000);
        if ~success
            fprintf('timed out waiting for data\n');
            continue;
        end
        
        %determine if we have been reset with a new fit.  if so,
        %  clear the state estimate covariance matrix
        if cam_getResetAndClear(ptr)
            fprintf('resetting filter\n');
            P = R;
            x = cam_getCurrentStateEstimate(ptr);  % get new estimate (from fit)
        else
            %TODO: perform update step; based on elapsed time
            x = cam_getCurrentStateEstimate(ptr);
            P = P + R;
        end
        prevX = x;
        
        [success, zk, id] = cam_getObservations(ptr);
        if ~success
            fprintf('error while attempting to get observations\n');
            continue;
        end
        
        %perform UKF
        thisQ = diag(repmat(Q, 1, numel(zk)/3));
        [x, P, sigmaX, sigmaZ] = unscented_update(@(x)cam_getExpectedObservations(ptr, x, id), [], ...
            x', P, zk', thisQ);
        
        % update the code with a new estimate
        cam_setCurrentStateEstimate(ptr, x);
        
        % draw some debug output
        figure(1);
        clf;
        ph = DrawAxis(GetHomoTransform(prevX), 0.1, 'b', 'b', 'k');
        hold on;
        ch = DrawAxis(GetHomoTransform(x), 0.1, 'b', 'b', 'r');
        z = reshape(zk, 3, [])';
        zh = plot3(z(:,1), z(:,2), z(:,3), 'sb');
        colors = 'bgykcm';
        legend_text = {'prev state', 'current state', 'observations'};
        ezh = [];
        for i = 1:length(id)
            ii = (i-1)*3+1;
            ci = mod(i-1, length(colors))+1;
            ezh(i) = plot3(sigmaZ(ii,:), sigmaZ(ii+1,:), sigmaZ(ii+2,:), ['.' colors(ci)]);
            legend_text{end+1} = sprintf('expected obs %d', id(i));
        end
        
        D = length(x);  % state dimension
        scale = 1;      % want scale = D+kappa == 3
        Ps = sqrt_posdef(P) * sqrt(scale);
        sigmaNewX = [x, repvec(x,D)+Ps, repvec(x,D)-Ps];        
        for i = 2:size(sigmaX,2)
            DrawAxis(GetHomoTransform(sigmaX(:,i)), 0.03, 'b', 'b', 'k');
            DrawAxis(GetHomoTransform(sigmaNewX(:,i)), 0.03, 'b', 'b', 'r');
        end
        
        hold off;
        legend([ph(3) ch(3) zh ezh], legend_text);
        grid on;
        axis equal;
        view(27,24);
        drawnow;
        1;
    end