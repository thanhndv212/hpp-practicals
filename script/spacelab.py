from hpp.corbaserver.manipulation.robot import Robot as Parent
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Constraints,
    ProblemSolver,
    Rule,
)
from hpp.gepetto import PathPlayer  # noqa: F401
from hpp.gepetto.manipulation import ViewerFactory

from hpp.corbaserver import loadServerPlugin
loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()

from pinocchio import Quaternion, SE3, SE3ToXYZQUAT

def XYZRPYToXYZQUAT(xyzrpy):
    from pinocchio.rpy import rpyToMatrix
    from pinocchio import SE3, SE3ToXYZQUAT
    import numpy
    xyz = numpy.array(xyzrpy[:3])
    rpy = numpy.array(xyzrpy[-3:])
    Matrix = rpyToMatrix(rpy)
    se3 = SE3(Matrix, xyz)
    return SE3ToXYZQUAT(se3)

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

class GroundDemo:
    rootJointType = "anchor"
    packageName = "spacelab_mock_hardware/description"
    urdfName = "ground_demo"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""

class ScrewDriver:
    rootJointType = "freeflyer"
    packageName = "spacelab_mock_hardware/description"
    urdfName = "screw_driver"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""

class FrameGripper:
    rootJointType = "freeflyer"
    packageName = "spacelab_mock_hardware/description"
    urdfName = "frame_gripper"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""

class CleatGripper:
    rootJointType = "freeflyer"
    packageName = "spacelab_mock_hardware/description"
    urdfName = "cleat_gripper"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""

class RS:
    rootJointType = "freeflyer"
    packageName = "spacelab_mock_hardware/description"
    urdfName = "RS"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""

robot = Robot("spacelab-robots", "spacelab")
ps = ProblemSolver(robot)
ps.setErrorThreshold(1e-4)
ps.setMaxIterProjection(40)

# Load Environment
vf = ViewerFactory(ps)
vf.loadEnvironmentModel(GroundDemo,"ground_demo")
vf.loadObjectModel(RS, "RS1")
vf.loadObjectModel(ScrewDriver, "screw_driver")
vf.loadObjectModel(FrameGripper, "frame_gripper")
vf.loadObjectModel(CleatGripper, "cleat_gripper")

q_ur10 = [0, 0, 0, 0, 0, 0]
q_vispa = [0, 0, 0, 0, 0, 0]
q_vispa2 = [0, 0]
q_RS1 = [0.46567999999999976,
         2.0219499999999999,
         -0.34200800000000015,
         1.5707938223931903,
         -3.1415918612707121,
         2.0943948257717535,
          ]
q_SD = [0.046500000888612281,
        1.3322769978599074,
        -1.2103000009948957,
        3.1415926529240905,
        -3.2988203261954171e-07,
        1.5707963267957175]
q_FG = [-0.13349999871947557,
        1.3322770029736009,
        -1.2132999986768855,
        -5.098912860707481e-09,
        -3.1415923241839976,
        1.5707963267958001]

q_CG = [0.04650005078371261,
        1.510277021366528,
        -1.212900037140666,
        4.1022589876578654e-07,
        3.1415906427953018,
        0.78539700669219625]

q_init = q_ur10 + q_vispa2 + q_vispa
for q_ in [q_RS1, q_SD, q_FG, q_CG]:
    q_init += XYZRPYToXYZQUAT(q_).tolist()

for joint in ["RS1/root_joint",
              "screw_driver/root_joint",
              "frame_gripper/root_joint",
              "cleat_gripper/root_joint"
              ]:
    robot.setJointBounds (joint, [-2,2,-3,3,-2,2,
                                -1.0001, 1.0001,-1.0001, 1.0001,
                                -1.0001, 1.0001,-1.0001, 1.0001,])


# Create constraint graph
constraints = []
# ps.createTransformationConstraint
grippers = ["spacelab/g_ur10_tool"]
objects = ["frame_gripper"]
handles_per_object = [["frame_gripper/h_FG_tool"]]

contactSurfacesPerObject = [[]]

envContactSurfaces = []

rules = [
    Rule([".*"], [".*"], True),
]

graph = ConstraintGraph(robot, "graph")
factory = ConstraintGraphFactory(graph)
factory.setGrippers(grippers)
factory.environmentContacts(envContactSurfaces)
factory.setObjects(objects, handles_per_object, contactSurfacesPerObject)
factory.setRules(rules)
factory.generate()
graph.addConstraints(graph=True, constraints=Constraints(numConstraints=constraints))
graph.initialize()

# set goals and solve
ps.setInitialConfig(q_init)
# ps.addGoalConfig(q_goal)
# ps.solve()

# Viewer
# v = vf.createViewer()
# v(q_init)
# pp = PathPlayer(v)

