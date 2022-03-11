from hpp.corbaserver.practicals.ur5 import Robot
from hpp.corbaserver import ProblemSolver, Client
from hpp.gepetto import ViewerFactory, PathPlayer
Client ().problem.resetProblem ()

robot = Robot ('ur5')
ps = ProblemSolver (robot)

vf = ViewerFactory (ps)

vf.loadObstacleModel ("package://hpp_practicals/urdf/ur_benchmark/obstacles.urdf",
                      "obstacles")
vf.loadObstacleModel ("package://hpp_practicals/urdf/ur_benchmark/table.urdf",
                      "table")
vf.loadObstacleModel ("package://hpp_practicals/urdf/ur_benchmark/wall.urdf",
                      "wall")

q1 = [0, -1.57, 1.57, 0, 0, 0]; q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]
q3 = [1.57, -1.57, -1.8, 0, 0.8, 0]

ps.setInitialConfig (q2)
ps.addGoalConfig (q3)

from motion_planner import MotionPlanner
m = MotionPlanner (robot, ps)
# pathId = m.solveBiRRT (maxIter = 1000)

v = vf.createViewer ()
v(q1)
# v (q3)
# pp = PathPlayer (v)
# pp (pathId)
