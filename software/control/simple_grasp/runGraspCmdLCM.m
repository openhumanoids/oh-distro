function runGraspCmdLCM()

megaclear
r = createSandiaManip();
c=ClosedGraspController(r);
runLCM(c)

end