function [contact_length, contact_width, contact_height] = contactVolume(biped, last_pos, next_pos, options)

if nargin < 4; options = struct(); end
if ~isfield(options, 'planar_clearance'); options.planar_clearance = 0.05; end
if ~isfield(options, 'nom_z_clearance'); options.nom_z_clearance = 0.05; end

next_pos(6) = last_pos(6) + angleDiff(last_pos(6), next_pos(6));

swing_angle = atan2(next_pos(2) - last_pos(2), next_pos(1) - last_pos(1));
phi.last = last_pos(6) - swing_angle;
phi.next = next_pos(6) - swing_angle;

contact_pts.last = quat2rotmat(axis2quat([0;0;1;phi.last])) * biped.foot_bodies.right.contact_pts;
contact_pts.next = quat2rotmat(axis2quat([0;0;1;phi.next])) * biped.foot_bodies.right.contact_pts; 
effective_width = max([max(contact_pts.last(2,:)) - min(contact_pts.last(2,:)),...
                       max(contact_pts.next(2,:)) - min(contact_pts.next(2,:))]);
effective_length = max([max(contact_pts.last(1,:)) - min(contact_pts.last(1,:)),...
                        max(contact_pts.next(1,:)) - min(contact_pts.next(1,:))]);
effective_height = (max([effective_length, effective_width])/2) / sqrt(2); % assumes the foot never rotates past 45 degrees in the world frame

% % We'll expand all of our obstacles in the plane by this distance, which is the maximum allowed distance from the center of the foot to the edge of an obstacle
contact_length = effective_length / 2 + options.planar_clearance;
contact_width = effective_width / 2 + options.planar_clearance;
contact_height = effective_height + options.nom_z_clearance;

end