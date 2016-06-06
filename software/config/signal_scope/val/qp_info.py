import numpy
import colorsys
# execfile(os.path.join(os.path.dirname(__file__), 'rotations.py'))



timeWindow = 5

# position plot
addPlot(timeWindow=timeWindow, yLimits=[0,3])
addSignal('CONTROLLER_STATE', msg.timestamp, msg.qpInfo)

