import os
drc_base_path = os.getenv("DRC_BASE")

import sys
sys.path.append(os.path.join(drc_base_path, "software", "models",
                             "model_transformation"))

import mitUrdfUtils as mit
from lxml import etree
from jointNameMap import jointNameMap

osrf_urdf_path = ("../components/osrf_original/atlas_v5_transmission.urdf")
mit_urdf_path = "../components/atlas_transmission_v5.urdf"

urdf = etree.parse(osrf_urdf_path)
mit.renameJoints(urdf, jointNameMap)

urdf.write(mit_urdf_path)
