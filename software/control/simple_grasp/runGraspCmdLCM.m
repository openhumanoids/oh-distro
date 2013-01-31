function runGraspCmdLCM()

megaclear
r = createSandiaManip();
c=ClosedGraspControllerInProg(r);
runLCM(c)

end