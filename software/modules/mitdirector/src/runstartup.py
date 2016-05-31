__author__ = 'manuelli'
sys.path.append(os.path.join(director.getDRCBaseDir(), 'software/modules'))

import mitdirector.src.startup
mitdirector.src.startup.startup(robotSystem, globals())
