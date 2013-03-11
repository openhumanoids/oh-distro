function links = vehicle_reader(mode)
if (mode==1)
  vehicle = xml2struct('/home/mfallon/.gazebo/models/polaris_ranger_ev/model-1_3.sdf')
else
  load vehicle
end

% chassis:
links = getLinks(vehicle);

%save('vehicle_and_links.mat','vehicle','links');
%keyboard


function links= getLinks(vehicle)
links=[]
for i =1:size( vehicle.Children(2).Children ,2 ) % 45 elements
  data = vehicle.Children(2).Children(i);
  if ( strcmp (data.Name, 'link' ) )
    link.data = data;
    link.name= link.data.Attributes.Value;
    link.pose= getLinkPose(link);
    link.collisions = getCollisions(link);
    links = [links;link]
  end
end


function pose = getLinkPose(part)
pose=[];
for i=1:size(part.data.Children,2)
  obj = part.data.Children(i);
  if ( strcmp (obj.Name,'pose') )
    pose = obj;
  end
end

function collisions = getCollisions(part)
% chassis name:
collisions=[];
for i=1:size(part.data.Children,2)
  obj = part.data.Children(i);
  if ( strcmp (obj.Name,'collision') )
    for j=1:size(obj.Children,2)
      if ( strcmp (obj.Children(j).Name, 'pose') )
        subpart.pose= obj.Children(j);
      end
      if ( strcmp (obj.Children(j).Name, 'geometry') )
        subpart.geometry = obj.Children(j);
      end
    end
    collisions = [collisions; subpart];
  end
end 
% chassis bottom
%vehicle.Children(2).Children(2).Children(6).Children(2).Children.Data