function URDF_Tester

% Construct robot and visualizer from urdf
%plant = RigidBodyManipulator('TaskWall.urdf');
plant = RigidBodyManipulator('URDF_Valve_Levers_Assembly.URDF');
visualizer = plant.constructVisualizer;
visualizer.inspector
end
