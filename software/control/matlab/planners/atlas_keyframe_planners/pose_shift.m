function x_cmd_new = pose_shift(x_des,x_cmd,x_meas,in_rpy)
if nargin < 4
  in_rpy=false;
end

% input  7x1 pose vectors for des, cmd and meas
% output 7x1 vector.
if(length(x_des)==6)
 x_des=convpose_rpy_to_quat(x_des);   
end
if(length(x_cmd)==6)
 x_cmd=convpose_rpy_to_quat(x_cmd);   
end
if(length(x_meas)==6)
 x_meas=convpose_rpy_to_quat(x_meas);   
end

x_cmd = x_cmd(:)';
x_meas = x_meas(:)';
x_des = x_des(:)';

x_cmd_new(1:3) = pos_shift(x_des(1:3),x_cmd(1:3),x_meas(1:3));
x_cmd_new(4:7) = quat_shift(x_des(4:7),x_cmd(4:7),x_meas(4:7));
x_cmd_new = x_cmd_new(:);

if(in_rpy)
  x_cmd_new=convpose_quat_to_rpy(x_cmd_new);
  x_cmd_new = x_cmd_new(:);
end


    function x_new=convpose_rpy_to_quat(x)        
        x_new = zeros(7,1);
        x_new(1:3) = x(1:3);
        x_new(4:7) =rpy2quat(x(4:6));
    end
    

    function x_new=convpose_quat_to_rpy(x)        
        x_new = zeros(6,1);
        x_new(1:3) = x(1:3);
        x_new(4:6) =quat2rpy(x(4:7));
    end    

    function   pos_cmd_new= pos_shift(pos_des,pos_cmd,pos_meas)
        pos_cmd = pos_cmd(:)';
        pos_meas = pos_meas(:)';
        pos_des = pos_des(:)';
        pos_cmd_new = pos_des + (pos_cmd - pos_meas);
        disp(pos_cmd - pos_meas);
    end


    function   quat_cmd_new= quat_shift(quat_des,quat_cmd,quat_meas)
        
        quat_cmd = quat_cmd(:)';
        quat_meas = quat_meas(:)';
        quat_des = quat_des(:)';
        
        % transformation that turn quat_meas tp quat_cmd
        qmeas_inv_qcmd = quatmultiply(quatinv(quat_meas),quat_cmd);
        
        % transform quat_des by the relative transformation between quat_mes
        % and quat_cmd
        quat_cmd_new = quatmultiply(quat_des,qmeas_inv_qcmd);
    end

    function  q_out = do_slerp(q1,q2,t)
        EPS = 1e-9;
        C = dot(q1,q2);
        
        if ((1 - C) <= EPS), % if angle is close by epsilon to 0 degrees -> calculate by linear interpolation
            q_out=q1*(1-t)+q2*t; % avoiding divisions by number close to 0
            q_out = quatnormalize(q_out);
            return;
        elseif ((1 + C) <= EPS), % when teta is close by epsilon to 180 degrees the result is undefined -> no shortest direction to rotate
            qtemp(1) = q2(4); qtemp(2) = -q2(3); qtemp(3)= q2(2); qtemp(4) = -q2(1); % rotating one of the unit quaternions by 90 degrees -> q2
            q2 =qtemp;
        end
        q1_inv    = quatinv(q1);
        q1_inv_q2 = quatmultiply(q1_inv,q2);
        omega     = quatlog(q1_inv_q2);
        q_out = quatmultiply(q1,quatexp(omega*t));
        %q_out = quatnormalize(q_out);
    end


end
