from math import sqrt
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
Client ().problem.resetProblem ()
from manipulation import robot, vf, ps, Ground, Box, Pokeball, PathPlayer, gripperName, ballName


# load enviroment for viewer
vf.loadEnvironmentModel (Ground, 'ground')
vf.loadEnvironmentModel (Box, 'box')
vf.moveObstacle ('box/base_link_0', [0.3+0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_1', [0.3-0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle ('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

vf.loadObjectModel (Pokeball, 'pokeball')
robot.setJointBounds ('pokeball/root_joint', [-.4,.4,-.4,.4,-.1,1.,
                                              -1.0001, 1.0001,-1.0001, 1.0001,
                                              -1.0001, 1.0001,-1.0001, 1.0001,])

q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

################## Planning constraint graph #################
## Create graph
graph = ConstraintGraph (robot, 'graph')

## Create nodes and edges
# Create nodes
graph.createNode (['grasp', 'ball-above-ground','grasp-placement', 
                'gripper-above-ball', 'placement'])

# Create edges
graph.createEdge ('placement', 'placement', 'transit', 1, 'placement')
graph.createEdge ('grasp', 'grasp', 'transfer', 1, 'grasp')

graph.createEdge ('placement', 'gripper-above-ball', 'approach-ball', 1, 'placement')
graph.createEdge ('gripper-above-ball', 'placement', 'move-gripper-away', 1, 'gripper-above-ball')

graph.createEdge ('gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'gripper-above-ball')
graph.createEdge ('grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'grasp-placement')

graph.createEdge ('grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp-placement')
graph.createEdge ('ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'ball-above-ground')

graph.createEdge ('ball-above-ground', 'grasp', 'take-ball-away', 1, 'ball-above-ground')
graph.createEdge ('grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp')


## Create transformation constraints

## configuration constraints

# grasp (gripper/ball): fixed all
ballInGripper = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint ('grasp', gripperName, ballName,
                                   ballInGripper, 6*[True,])

# placement (world/ball): fixed z, fixed r, p
ps.createTransformationConstraint ('placement', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [False, False, True, True, True, False,])

# gripper/inside_box: fixed all
# ps.createTransformationConstraint ('gripper/inside-box', '', gripperName,
#                                    [0.3,0,0.025,0.5, 0.5, -0.5, 0.5],
#                                    6*[True])
## motion constraints
# placement/complement (world/ball): fixed x and y, fixed y
ps.createTransformationConstraint ('placement/complement', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [True, True, False, False, False, True,])

# gripper/aligned (world/gripper): fixed x and y, fixed r, p ,y  
ps.createTransformationConstraint ('gripper/aligned', '', gripperName,
                                   [0.3, 0., 0., 0.5, 0.5, -0.5, 0.5],
                                   [True, False, True, True, True, True,])


## Add constraints to nodes
graph.addConstraints (node='placement', constraints = \
                      Constraints (numConstraints = ['placement'],))

graph.addConstraints (node='gripper-above-ball', constraints = \
                      Constraints (numConstraints = ['placement/complement', 'gripper/aligned'],))

graph.addConstraints (node='grasp-placement', constraints = \
                      Constraints (numConstraints = ['grasp', 'gripper/aligned'],))

graph.addConstraints (node='ball-above-ground', constraints = \
                      Constraints (numConstraints = ['grasp', 'gripper/aligned'],))

graph.addConstraints (node='grasp', constraints = \
                      Constraints (numConstraints = ['grasp'],))

## Add constraints to edges
graph.addConstraints (edge='transit', constraints = \
                      Constraints (numConstraints = ['placement/complement']))


graph.addConstraints (edge='approach-ball', constraints = \
                      Constraints (numConstraints = ['placement/complement']))

graph.addConstraints (edge='grasp-ball', constraints = \
                      Constraints (numConstraints = ['placement/complement', 'gripper/aligned']))

graph.addConstraints (edge='take-ball-up', constraints = \
                      Constraints (numConstraints = ['grasp', 'gripper/aligned']))

graph.addConstraints (edge='take-ball-away', constraints = \
                      Constraints (numConstraints = ['grasp']))

graph.addConstraints (edge='approach-ground', constraints = \
                      Constraints (numConstraints = ['grasp']))

graph.addConstraints (edge='put-ball-down', constraints = \
                      Constraints (numConstraints = ['grasp', 'gripper/aligned']))

graph.addConstraints (edge='move-gripper-up', constraints = \
                      Constraints (numConstraints = ['grasp', 'gripper/aligned']))

graph.addConstraints (edge='move-gripper-away', constraints = \
                      Constraints ())

graph.addConstraints (edge='transfer',     constraints = Constraints ())


## Configure problem solver 
ps.selectPathValidation ("Discretized", 0.01)
ps.selectPathProjector ("Progressive", 0.1)

ps.setConstantRightHandSide ('placement', True)
ps.setConstantRightHandSide ('placement/complement', False)


## Initialize graph
graph.initialize ()
graph.display (format='svg')
################# Define and solve manipulation problem ######


res, q_init, error = graph.applyNodeConstraints ('placement', q1)
q2 = q1 [::]
q2 [7] = .2

res, q_goal, error = graph.applyNodeConstraints ('placement', q2)

ps.setInitialConfig (q_init)
ps.addGoalConfig (q_goal)
ps.solve()

################ Viewer display #############################
v = vf.createViewer ()
pp = PathPlayer (v)
# v (q_goal)
pp(0)
