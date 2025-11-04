import numpy as np
import pinocchio as pin
from pinocchio import SE3
from pyhpp.pinocchio import Device, urdf
from pyhpp.core import (
    Problem,
    DiffusingPlanner,
    BiRRTPlanner,
    VisibilityPrmPlanner,
    BiRrtStar,
    kPrmStar,
)
from pyhpp.gepetto.viewer import Viewer

robot_urdf = (
    "package://spacelab_mock_hardware/description/urdf/allRobots_spacelab_robot.urdf"
)
robot_srdf = (
    "package://spacelab_mock_hardware/description/srdf/allRobots_spacelab_robot.srdf"
)
scene_urdf = (
    "package://spacelab_mock_hardware/description/urdf/gd_scene.urdf"
)
# scene_srdf = ("")

# Load robots and scene in pyhpp
robot = Device.create("spacelab")
urdf.loadModel(robot, 0, "spacelab-robots", "anchor", robot_urdf, robot_srdf, SE3.Identity())
urdf.loadModel(robot, 0, "spacelab-scene", "anchor", scene_urdf, "", SE3.Identity())

# Load in viewer
v = Viewer(robot)
v(pin.neutral(robot.model()))