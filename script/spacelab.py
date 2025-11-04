from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import Client, ProblemSolver
from hpp.gepetto import PathPlayer  # noqa: F401
from hpp.gepetto.manipulation import ViewerFactory

from hpp.corbaserver import loadServerPlugin
loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

class Robot(Parent):
    packageName = "spacelab_mock_hardware/description"
    urdfName = "allRobots_spacelab_robot"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, compositeName, robotName, load=True, rootJointType="anchor"):
        Parent.__init__(self, compositeName, robotName, rootJointType, load)

class LabScene:
    rootJointType = "anchor"
    packageName = "spacelab_mock_hardware/description"
    urdfName = "gd_scene"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""

robot = Robot("spacelab-robots", "spacelab")
ps = ProblemSolver(robot)
ps.setErrorThreshold(1e-4)
ps.setMaxIterProjection(40)

vf = ViewerFactory(ps)
vf.loadEnvironmentModel(LabScene,"labscene")

v = vf.createViewer()
v(robot.shootRandomConfig())
