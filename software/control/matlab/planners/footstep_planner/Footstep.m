classdef Footstep
  properties
    pos
    id
    body_idx
    is_in_contact
    pos_fixed
    terrain_pts
    infeasibility
    walking_params
    frames
  end

  properties(Constant)
    atlas_foot_bodies_idx = struct('right', 28, 'left', 16);
  end

  methods
    function obj = Footstep(biped, pos, id, body_idx, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params, frames)
      if nargin < 10
        obj.frames = struct('orig', CoordinateFrame('orig', 6, 'o', {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}),...
                            'center', CoordinateFrame('center', 6, 'c', {'x', 'y', 'z', 'roll', 'pitch', 'yaw'}));
        if body_idx == biped.foot_bodies_idx.right
          offset = biped.foot_contact_offsets.right.center;
        elseif body_idx == biped.foot_bodies_idx.left
          offset = biped.foot_contact_offsets.left.center;
        else
          error('Don''t know how to handle body indices other than right/left feet');
        end
        obj.frames.orig.addTransform(FootstepContactTransform(obj.frames.orig, ...
                                                              obj.frames.center,...
                                                              offset));
        obj.frames.center.addTransform(FootstepContactTransform(obj.frames.center, ...
                                                              obj.frames.orig,...
                                                              -offset));
      else
        obj.frames = frames;
      end
      obj.pos = Point(obj.frames.orig, pos);
      obj.id = id;
      obj.body_idx = body_idx;
      obj.is_in_contact = is_in_contact;
      obj.pos_fixed = pos_fixed;
      obj.terrain_pts = terrain_pts;
      obj.infeasibility = infeasibility;
      obj.walking_params = walking_params;
    end

    function msg = to_footstep_t(obj)
      msg = drc.footstep_t();
      msg.pos = encodePosition3d(obj.pos.inFrame(obj.frames.orig));
      msg.id = obj.id;
      if obj.body_idx == Footstep.atlas_foot_bodies_idx.right
        msg.is_right_foot = true;
      elseif obj.body_idx == Footstep.atlas_foot_bodies_idx.left
        msg.is_right_foot = false;
      else
        error('DRC:Footstep:InvalidBodyIdxForAtlas', 'Invalid body index for Atlas robot');
      end
      msg.is_in_contact = obj.is_in_contact;
      msg.fixed_x = obj.pos_fixed(1);
      msg.fixed_y = obj.pos_fixed(2);
      msg.fixed_z = obj.pos_fixed(3);
      msg.fixed_roll = obj.pos_fixed(4);
      msg.fixed_pitch = obj.pos_fixed(5);
      msg.fixed_yaw = obj.pos_fixed(6);
      msg.num_terrain_pts = size(obj.terrain_pts, 2);
      if msg.num_terrain_pts > 0
        msg.terrain_path_dist = obj.terrain_pts(1,:);
        msg.terrain_height = obj.terrain_pts(2,:);
      end
      msg.infeasibility = obj.infeasibility;
      msg.params = obj.walking_params;
    end
  end

  methods(Static=true)
    function footstep = from_footstep_t(msg, biped)
      pos = decodePosition3d(msg.pos);
      id = msg.id;
      if msg.is_right_foot
        body_idx = Footstep.atlas_foot_bodies_idx.right;
      else
        body_idx = Footstep.atlas_foot_bodies_idx.left;
      end
      is_in_contact = msg.is_in_contact;
      pos_fixed = [msg.fixed_x;
                   msg.fixed_y;
                   msg.fixed_z;
                   msg.fixed_roll;
                   msg.fixed_pitch;
                   msg.fixed_yaw];
      terrain_pts = [reshape(msg.terrain_path_dist, 1, []);
                     reshape(msg.terrain_height, 1, []);];
      infeasibility = msg.infeasibility;
      walking_params = msg.params;
      footstep = Footstep(biped, pos, id, body_idx, is_in_contact, pos_fixed, terrain_pts, infeasibility, walking_params);
    end
  end
end




