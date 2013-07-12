group "Drake-only" {
    cmd "PendulumEnergyControl" {
        exec = "matlab -nosplash -nodesktop -r \"addpath([getDrakePath(),'/examples/Pendulum']); c=PendulumEnergyControl(setInputLimits(PendulumPlant,-inf,inf)); runLCM(c);\"";
        host = "localhost";
    }
    cmd "PendulumPlant" {
        exec = "matlab -nosplash -nodesktop -r \"addpath([getDrakePath(),'/examples/Pendulum']); p=PendulumPlant(); runLCM(p);\" ";
        host = "localhost";
    }
    cmd "PendulumVisualizer" {
        exec = "matlab -nosplash -nodesktop -r \"addpath([getDrakePath(),'/examples/Pendulum']); v=PendulumVisualizer(); runLCM(v);\"";
        host = "localhost";
    }
}

group "DRC viewer" {
    cmd "viewer" {
        exec = "drc-viewer -c drc_robot.cfg";
        host = "localhost";
    }
    cmd "urdf publisher" {
        exec = "robot_model_publisher ../../drake/examples/Pendulum/Pendulum.urdf";
        host = "localhost";
    }
}


