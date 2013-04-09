function [val] =  quat_spline(q,s,mode)
% Implements a  spline interpolation (slerp) of N quaternions in spherical space of SO(3) (3-Sphere).
% q is a vector of size 4 x N where N is the number unit quaternion vectors
% s is a scalar from 0-1 that indicates where along the quaternion array the
% interpolation is happening.
% Two modes of spline interpolation
% a) 'hermite_cubic':cubic  curve between consequtive points using
% Bezier-Bernstein basis (see Ref[1])
% b) 'squad' (This is Default) :[s]pherical and [quad]rangle interpolation (see Ref[2] &
% [3]). Analogous to bilinear interpolation in euclidean space.
% squad is a sequence of  hierachical slerp interpolations.
% squad(t,p,a,b,q) = slerp(2t(1-t),slerp(t,p,q),slerp(t,a,b));

% Implmentation  based on the following papers.

% [1] Kim, M. J., Kim, M. S., & Shin, S. Y. (1995, September). A general
% construction scheme for unit quaternion curves with simple high order
% derivatives. In Proceedings of the 22nd annual conference on Computer
% graphics and interactive techniques (pp. 369-376). ACM.

% Choice of end velocites isb ased on the following resources.
% [2] Dam, Erik B., Martin Koch, and Martin Lillholm. Quaternions,
% interpolation and animation. Datalogisk Institut, Københavns Universitet, 1998.
% http://web.mit.edu/2.998/www/QuaternionReport1.pdf

% [3] Eberly, David. "Quaternion algebra and calculus." Magic Software, Inc 21 (2002).
% http://www.geometrictools.com/Documentation/Quaternions.pdf



if(nargin~=3)
    mode ='squad';
end

D = size(q,1);
L = size(q,2);
val = q(:,1)';
j=1;
order = 3;


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
    alpha=eval_alpha(s,j,L); % shifting global percentage value to interval percentage.
    t= alpha;
    

    if(alpha>0)
        EPS = 1e-9;
        % Compute the cosine of the angle between the two vectors.
        C = dot(q(:,j-1),q(:,j));
        %disp(C>0)
        if ((1 - C) <= EPS), % if angle is close by epsilon to 0 degrees -> calculate by linear interpolation
            val=q(:,j-1)'*(1-s)+q(:,j)'*s; % avoiding divisions by number close to 0
            val = quatnormalize(val);
        elseif ((1 + C) <= EPS), % when teta is close by epsilon to 180 degrees the result is undefined -> no shortest direction to rotate
            qtemp(1) = q(4,j); qtemp(2) = -q(3,j); qtemp(3)= q(2,j); qtemp(4) = -q(1,j); % rotating one of the unit quaternions by 90 degrees -> q2
            
            qtemp_array = q;
            qtemp_array(:,j) = qtemp';
            
            if(strcmp(mode,'hermite_cubic'))
                qi=quatinv(q(:,j-1)');
                qa = get_intermediate_control_point(j-1,qtemp_array,0);
                qap1 = get_intermediate_control_point(j,qtemp_array,1);
                qai =quatinv(qa);
                qap1i =quatinv(qap1);
                qiqa=quatmultiply(qi,qa);
                qaiqap1=quatmultiply(qai,qap1);
                qap1iqp1=quatmultiply(qap1i,qtemp);
                omega1 = quatlog(qiqa);
                omega2 = quatlog(qaiqap1);
                omega3 = quatlog(qap1iqp1);
                beta1 = eval_cumulative_berstein_basis(t,1,order);
                beta2 = eval_cumulative_berstein_basis(t,2,order);
                beta3 = eval_cumulative_berstein_basis(t,3,order);
                val=quatmultiply(q(:,j-1)',quatexp(omega1*beta1));
                val=quatmultiply(val,quatexp(omega2*beta2));
                val=quatmultiply(val,quatexp(omega3*beta3));
                val = quatnormalize(val);
            elseif(strcmp(mode,'squad'))
                qa = get_intermediate_control_point(j-1,qtemp_array,0);
                qap1 = get_intermediate_control_point(j,qtemp_array,0);
                qtemp1 = do_slerp(q(:,j-1)', qtemp, t);
                qtemp2 = do_slerp(qa, qap1, t);
                squad = do_slerp(qtemp1, qtemp2, 2*t*(1-t));
                val = squad;val = quatnormalize(val);
            end
            
        else
            
            if(strcmp(mode,'hermite_cubic'))
                qi=quatinv(q(:,j-1)');
                qa = get_intermediate_control_point(j-1,q,0);
                qap1 = get_intermediate_control_point(j,q,1);
                qai =quatinv(qa);
                qap1i =quatinv(qap1);
                qiqa=quatmultiply(qi,qa);
                qaiqap1=quatmultiply(qai,qap1);% (qa(n)^-1)qa(n+1)
                qap1iqp1=quatmultiply(qap1i,q(:,j)');
                omega1 = quatlog(qiqa);
                omega2 = quatlog(qaiqap1);
                omega3 = quatlog(qap1iqp1);
                beta1 = eval_cumulative_berstein_basis(t,1,order);
                beta2 = eval_cumulative_berstein_basis(t,2,order);
                beta3 = eval_cumulative_berstein_basis(t,3,order);
                val=quatmultiply(q(:,j-1)',quatexp(omega1*beta1));
                val=quatmultiply(val,quatexp(omega2*beta2));
                val=quatmultiply(val,quatexp(omega3*beta3));
                val = quatnormalize(val);
            elseif(strcmp(mode,'squad'))
                qa = get_intermediate_control_point(j-1,q,0);
                qap1 = get_intermediate_control_point(j,q,0);
                qtemp1 = do_slerp(q(:,j-1)', q(:,j)', t);
                qtemp2 = do_slerp(qa, qap1, t);
                squad = do_slerp(qtemp1, qtemp2, 2*t*(1-t));
                val = squad;val = quatnormalize(val);
            end
        end

    end

end

end

function [qa] = get_intermediate_control_point(j,q,dir_flip)
% intermediate quaternions for spline continuity (the squad endpt conditions)
% see page 54 of
% Dam, Erik B., Martin Koch, and Martin Lillholm. Quaternions,
% interpolation and animation. Datalogisk Institut, Københavns Universitet, 1998.
% http://web.mit.edu/2.998/www/QuaternionReport1.pdf

L = size(q,2);
if(j==1)
    qa =  q(:,1)';
    return;
elseif(j==L)
    qa =  q(:,L)';
    return;
else
    qji=quatinv(q(:,j)');
    qiqm1=quatmultiply(qji,q(:,j-1)');
    qiqp1=quatmultiply(qji,q(:,j+1)');
    ang_vel =-((quatlog(qiqp1)+quatlog(qiqm1))/4); %average of end pt tangents
    if(dir_flip)
        qa = quatmultiply(q(:,j)',quatinv(quatexp(ang_vel)));
    else
        qa = quatmultiply(q(:,j)',quatexp(ang_vel));
    end
    %qa = quatnormalize(qa);
end
end

function beta = eval_cumulative_berstein_basis(s,i,order)

N=order;
beta = 0;
for j=i:N,
    term1 = factorial(N)/(factorial(j)*factorial(N-j));
    term2 =(1-s)^(N-j);
    term3 =(s)^(j);
    beta = beta+ term1*term2*term3;
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

