function trajectoryTrackingTestAnalysis(file_name, pose_name)
  document = xmlread(file_name);  
  results = document.getDocumentElement();
  test_list = results.getElementsByTagName('test');
  for i = 0:test_list.getLength() - 1
    if strcmp(char(test_list.item(i).getAttribute('name')), pose_name)
      test = test_list.item(i);
      break
    end
  end
  plan_node = test.getElementsByTagName('committed_plan').item(0);
  plan_element_list = plan_node.getElementsByTagName('position');
  
  execution_node = test.getElementsByTagName('executed_plan').item(0);  
  execution_element_list = execution_node.getElementsByTagName('position');
  
  plan_time_string = char(plan_node.getElementsByTagName('time').item(0).getFirstChild().getNodeValue());
  plan_time = sscanf(plan_time_string, '%d') / 1e6;
  execution_time_string = char(execution_node.getElementsByTagName('time').item(0).getFirstChild().getNodeValue());
  execution_time = double(sscanf(execution_time_string, '%g')) / 1e6;
  execution_time = (execution_time - execution_time(1));
  
  execution_time_start = find(execution_time >= execution_time(end)-plan_time(end), 1);
  execution_time = execution_time(execution_time_start:end) - execution_time(execution_time_start);
  
  clf;
  
  for joint = 0:plan_element_list.getLength()-1
    joint_name = plan_element_list.item(joint).getAttribute('joint_name');
    plan_position_string = char(plan_element_list.item(joint).getFirstChild().getNodeValue());
    plan_position = sscanf(plan_position_string, '%g');
    for j = 0:execution_element_list.getLength()-1
      if strcmp(execution_element_list.item(j).getAttribute('joint_name'), joint_name)
        execution_position_string = char(execution_element_list.item(j).getFirstChild().getNodeValue());
        execution_position = sscanf(execution_position_string, '%g');
        execution_position = execution_position(execution_time_start:end);
        break;
      end
    end
    subplot(4, 8, joint+1)
    plot(plan_time, plan_position, 'b')
    hold on
    plot(execution_time, execution_position, 'r')
    title(char(joint_name))
  end
end