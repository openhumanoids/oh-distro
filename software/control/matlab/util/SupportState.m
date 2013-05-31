classdef SupportState
  %SUPPORTSTATE defines a set of supporting bodies and contacts for a rigid
  %  body manipulator
  
  properties (SetAccess=protected)
    bodies; % array of supporting body indices
    contact_pts; % cell array of supporting contact point indices
    num_contact_pts; % convenience array containing the desired number of 
                     %   contact points for each support body
  end
  
  methods
    function obj = SupportState(r,bodies,contact_pts)
      typecheck(r,'Atlas');
      typecheck(bodies,'double');
      obj.bodies = bodies(bodies~=0);
      
      obj.num_contact_pts=zeros(length(obj.bodies),1);
      if nargin>2
        typecheck(contact_pts,'cell');
        sizecheck(contact_pts,length(obj.bodies));
        for i=1:length(obj.bodies)
          % verify that contact point indices are valid
          if any(contact_pts{i}>size(getBodyContacts(r,obj.bodies(i)),2))
            error('SupportState: contact_pts indices exceed number of contact points');
          end
          obj.num_contact_pts(i)=length(contact_pts{i});
        end
        obj.contact_pts = contact_pts;
      else
        % use all points on body
        obj.contact_pts = cell(length(obj.bodies),1);
        for i=1:length(obj.bodies)
          obj.contact_pts{i} = 1:size(getBodyContacts(r,obj.bodies(i)),2);
          obj.num_contact_pts(i)=length(obj.contact_pts{i});
        end
      end
    end
  end
  
end

