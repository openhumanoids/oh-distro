__author__ = 'manuelli'

import contactfilter
import contactfiltervisualizer
import externalforce
import linkselection


def startup(robotSystem, globalsDict=None):
    print "setting up the contact particle filter . . . "
    rs = robotSystem

    externalForce = externalforce.ExternalForce(rs)
    contactFilter = contactfilter.ContactFilter(rs.robotStateModel, rs.robotStateJointController)
    contactFilterVisualizer = contactfiltervisualizer.ContactFilterVisualizer(rs, rs.robotStateModel)
    linkSelection = linkselection.LinkWidget(rs.view, rs.robotStateModel, externalForce)

    if globalsDict is not None:
        globalsDict['externalForce'] = externalForce
        globalsDict['contactFilter'] = contactFilter
        globalsDict['contactFilterVisualizer'] = contactFilterVisualizer
        globalsDict['linkSelection'] = linkSelection

    print "contact particle filter initialized "
