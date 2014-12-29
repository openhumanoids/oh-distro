#!/usr/bin/python
import os
drc_base_path = os.getenv("DRC_BASE")

import sys
sys.path.append(os.path.join(drc_base_path, "software", "models",
                             "model_transformation"))

import mitUrdfUtils as mit

os.chdir(os.path.dirname(os.path.realpath(__file__)))
mit.xacro("atlas_v4_LR_RR.xacro", "../model_LR_RR.urdf", verbose=True)
mit.xacro("atlas_v4_minimal_contact.xacro", "../model_minimal_contact.urdf",
          verbose=True)
mit.xacro("atlas_v4_convex_hull_open_hands.xacro", "../model_convex_hull.urdf",
          verbose=True)
mit.xacro("atlas_v4_convex_hull_closed_hands.xacro",
          "../model_convex_hull_closed_hands.urdf", verbose=True)
