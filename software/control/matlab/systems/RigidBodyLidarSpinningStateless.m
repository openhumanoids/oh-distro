classdef RigidBodyLidarSpinningStateless < RigidBodyDepthSensor
  
  methods
    
    function obj = RigidBodyLidarSpinningStateless(lidar_name, frame_id, min_yaw, max_yaw, num_pixels, range, spin_rate)
      % @param lidar_name name of  this lidar (as a string)
      % @param frame_id ID of the RigidBodyFrame
      % @param min_yaw minimum yaw angle (in radians) that the sensor
      %   reads
      % @param max_yaw maximum yaw angle (in radians) that the sensor
      %   reads
      % @param num_pixels number of pixels that the sensor reads
      % @param range maximum range of the laser (in meters).  can be inf.
      % @param spinrate Spinning rate in rad/sec
      obj = obj@RigidBodyDepthSensor(lidar_name,frame_id,0,0,1,min_yaw,max_yaw,num_pixels,range);
      obj.spin_rate = spin_rate;
    end
    
    function points = output(obj,manip,t,x,u)
      % Computes the output of the sensor
      % @param manip RigidBodyManipulator with collision geometries
      % @param t time (unused)
      % @param x system state
      % @param u control input (unused)
      %
      % @retval distance array of distances returned by the sensor, size
      %  will equal the pixel width of yaw scan, prepending with the
      % current spindle angle.
      
      amount_spun = t*obj.spin_rate;
      % spin bodypoints by that amount around the +z axis
      rot_matrix = [1, 0, 0;
                    0, cos(amount_spun), -sin(amount_spun);
                    0, sin(amount_spun), cos(amount_spun);];
                  
      rot_body_points = rot_matrix * obj.body_pts;
      
      % compute raycasting points
      
      kinsol = doKinematics(manip,x(1:getNumPositions(manip)));
      pts = forwardKin(manip,kinsol,obj.frame_id,[zeros(3,1),rot_body_points]);
      
      origin = repmat(pts(:,1),1,obj.num_pixel_rows*obj.num_pixel_cols);
      point_on_ray = pts(:,2:end);
      
      distance = collisionRaycast(manip, kinsol, origin, point_on_ray, false);
      distance( distance<0 ) = obj.range;
      
      if (~isempty(obj.lcmgl))
        points = (repmat(distance',3,1)/obj.range).*rot_body_points;
        points_in_world_frame = forwardKin(manip,kinsol,obj.frame_id,points);
        if (size(points_in_world_frame, 2) > 0)
          obj.lcmgl.glColor3f(1, 0, 0);
          obj.lcmgl.points(points_in_world_frame(1,:),points_in_world_frame(2,:),points_in_world_frame(3,:));
          %          obj.lcmgl.line3(origin(1), origin(2), origin(3), point(1,i), point(2,i), point(3,i));
          for i=1:length(points_in_world_frame(1, :))
            obj.lcmgl.sphere([points_in_world_frame(1,i),points_in_world_frame(2,i),points_in_world_frame(3,i)], 0.05, 20, 20);
          end
          obj.lcmgl.switchBuffers;
        end
      end
      
      points = distance; % Take norm of each column to get distance
      points = [mod(amount_spun, 2*3.1415); points]; % Tack on spindle position
      
    end
    
    function fr = constructFrame(obj,manip)
      fr = CoordinateFrame(obj.name,obj.num_pixel_cols + 1,'d');
    end  
    
  end
  
  properties
    spin_rate;
  end

end