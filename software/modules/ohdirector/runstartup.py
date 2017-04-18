sys.path.append(os.path.join(director.getDRCBaseDir(), 'software/modules'))

import ohdirector.startup
ohdirector.startup.startup(robotSystem, globals())
