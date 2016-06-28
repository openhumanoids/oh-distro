__author__ = 'manuelli'
sys.path.append(os.path.join(director.getDRCBaseDir(), 'software/control/residual_detector/python/src'))

import contactparticlefilterstartup
contactparticlefilterstartup.startup(robotSystem, globals())
