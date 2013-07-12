function runGraspControllerLCM()

megaclear
[r_left,r_right] = createSandiaManips();
c = SandiaGraspController(r_left,r_right);

options.timekeeper = 'drake/lcmTimeKeeper'; % Synced to LCM input messages. 
runLCM(c,[],options);
end
