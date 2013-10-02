function [val] =  quat_foh(q,s)
% Implements piece wise spherical interpolation (slerp) of N quaternions .

% q is a vector of size 4 x N where N is the number unit quaternion vectors
% s is a scalar from 0-1 that indicates where along the quaternion array the
% interpolation is happening.

% Based on the following paper.
%-------------------------------------------------
% [1] Kim, M. J., Kim, M. S., & Shin, S. Y. (1995, September). A general
% construction scheme for unit quaternion curves with simple high order
% derivatives. In Proceedings of the 22nd annual conference on Computer
% graphics and interactive techniques (pp. 369-376). ACM.



D = size(q,1);
L = size(q,2);
val = q(:,1)';
j=1;
alpha = eval_alpha(s,j,L);

for j=2:L,
    C = dot(q(:,j-1),q(:,j));
    if(C<0)
        q(:,j) = -q(:,j);
    end 
end

C = dot(q(:,1),q(:,end));        
if s==0 % saving calculation time -> where qm=qi
    val=q(:,1);
    return;
elseif s==1 % saving calculation time -> where qm=qn
    val=q(:,end);
    return;
end


for j =2:L,
    alpha = eval_alpha(s,j,L);
    
    if(alpha>0)
        
        EPS = 1e-9;
        % Compute the cosine of the angle between the two vectors.
        C = dot(q(:,j-1),q(:,j));
        %disp(C>=0)
        if ((1 - C) <= EPS), % if angle is close by epsilon to 0 degrees -> calculate by linear interpolation
            val=q(:,j-1)'*(1-s)+q(:,j)'*s; % avoiding divisions by number close to 0
            val = quatnormalize(val);
        elseif ((1 + C) <= EPS), % when teta is close by epsilon to 180 degrees the result is undefined -> no shortest direction to rotate
            qtemp(1) = q(4,j); qtemp(2) = -q(3,j); qtemp(3)= q(2,j); qtemp(4) = -q(1,j); % rotating one of the unit quaternions by 90 degrees -> q2
            qi=quatinv(q(:,j-1)');
            qiq=quatmultiply(qi,qtemp);
            omega=quatlog(qiq);
            val=quatmultiply(val,quatexp(omega*alpha));
            val = quatnormalize(val);
        else
            qi=quatinv(q(:,j-1)');
            qiq=quatmultiply(qi,q(:,j)');
            omega=quatlog(qiq);
            val=quatmultiply(val,quatexp(omega*alpha));
            val = quatnormalize(val);
        end
        
        
        
    end
end

end


function alpha = eval_alpha(s,i,L)
k = s*(L-1)+1; % shifting percentage value to interval index.

if(i<=k)
    alpha=1;
elseif((i>k)&&(i<k+1))
    alpha=k-(i-1);
else
    alpha=0;
end

end
