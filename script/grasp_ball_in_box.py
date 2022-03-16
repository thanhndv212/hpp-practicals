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

## motion constraints
# placement/complement (world/ball): fixed x and y, fixed yaw
ps.createTransformationConstraint ('placement/complement', '', ballName,
                                   [0,0,0.025,0, 0, 0, 1],
                                   [True, True, False, False, False, True,])

# gripper_ball_aligned (world/gripper): fixed x and y, fixed r, p ,y  
ps.createTransformationConstraint ('gripper_ball_aligned', gripperName, ballName,
                                   [0, .2, 0 , 0.5, 0.5, -0.5, 0.5],
                                   [True, True , True, True, True, True,])

# ball in gripper and near table: 
ps.createTransformationConstraint ('ball_near_table', '', ballName,
                                   [0,0,0.1,0, 0, 0, 1],
                                   [True, True, True, True, True, True,])

## Ball on table
graph.addConstraints (node='placement', constraints = \
                      Constraints (numConstraints = ['placement'],))
graph.addConstraints (edge='transit', constraints = \
                      Constraints (numConstraints = ['placement/complement']))

graph.createEdge ('placement', 'gripper-above-ball', 'approach-ball', 1, 'placement')
graph.createEdge ('gripper-above-ball', 'placement', 'move-gripper-away', 1, 'placement')
graph.addConstraints (edge='approach-ball', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-away', constraints = \
                      Constraints ())

## Gripper above ball, ball on table
graph.addConstraints (node='gripper-above-ball', constraints = \
                      Constraints (numConstraints = ['placement', 'gripper_ball_aligned'],))

graph.createEdge ('gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'placement')
graph.createEdge ('grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'placement')
graph.addConstraints (edge='grasp-ball', constraints = \
                      Constraints (numConstraints = ['placement/complement']))
graph.addConstraints (edge='move-gripper-up', constraints = \
                      Constraints (numConstraints = ['placement/complement']))

## Gripper grasp ball, ball on table
graph.addConstraints (node='grasp-placement', constraints = \
                      Constraints (numConstraints = ['grasp', 'placement'],))
                      
graph.createEdge ('grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp')
graph.createEdge ('ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'grasp')
graph.addConstraints (edge='take-ball-up', constraints = \
                      Constraints ())
graph.addConstraints (edge='put-ball-down', constraints = \
                      Constraints ())

## Gripper grasp ball, ball near table
graph.addConstraints (node='ball-above-ground', constraints = \
                      Constraints (numConstraints = ['grasp', 'ball_near_table'],))

graph.createEdge ('ball-above-ground', 'grasp', 'take-ball-away', 1, 'grasp')
graph.createEdge ('grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp')
graph.addConstraints (edge='take-ball-away', constraints = \
                      Constraints ())
graph.addConstraints (edge='approach-ground', constraints = \
                      Constraints ())


## Gripper grasp ball
graph.addConstraints (node='grasp', constraints = \
                      Constraints (numConstraints = ['grasp'],))
graph.addConstraints (edge='transfer',     constraints = Constraints ())
## Add constraints to edges

## Configure problem solver 
ps.selectPathValidation ("Discretized", 0.01)
# ps.selectPathValidation ("Dichotomy", 0)

ps.selectPathProjector ("Progressive", 0.1)

ps.setConstantRightHandSide ('placement', True)
ps.setConstantRightHandSide ('placement/complement', False)
ps.setConstantRightHandSide
print("Start constructing constraint graph ... ")
## Initialize graph
graph.initialize ()
################# Define and solve manipulation problem ######
# (['grasp', 'ball-above-ground','grasp-placement', 
                # 'gripper-above-ball', 'placement'])



# Test nodes and edges 
import numpy as np

res, q_init, error = graph.applyNodeConstraints ('placement', q1)
print("Initial configuration: ",np.around(q_init,4))

for i in range(100):
    q = robot.shootRandomConfig()
    res, q_ab , err = graph.generateTargetConfig('approach-ball',q1,q)
    if res: break
print("Configuration after approach-ball: ", np.around(q_ab, 4))
res, q_gab, err = graph.applyNodeConstraints('gripper-above-ball',q_ab)
print("Congiguration at gripper above ball: ",np.around(q_gab,4))

# for i in range(100):
#     q = robot.shootRandomConfig()
#     res, q_mga , err = graph.generateTargetConfig('move-gripper-away',q_gab, q1)
#     if res: break
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_gb , err = graph.generateTargetConfig('grasp-ball',q_gab,q)
    if res: break
print("Configuration after grasp ball: ",np.around(q_gb, 4))

res, q_gp, err = graph.applyNodeConstraints('grasp-placement',q_gb)
print("Configuration at grasp placement: ", np.around(q_gp,4))

# for i in range(100):
#     q = robot.shootRandomConfig()
#     res, q_mgu , err = graph.generateTargetConfig('move-gripper-up',q_gp,q)
#     if res: break

# print("Configuration after move gripper up: ", np.around(q_mgu, 4))

for i in range(100):
    q = robot.shootRandomConfig()
    res, q_tbu , err = graph.generateTargetConfig('take-ball-up',q_gp,q)
    if res: break
print("Configuration after take ball up: ",np.around(q_tbu, 4))

res, q_bag, err = graph.applyNodeConstraints('ball-above-ground',q_tbu)
print("Configuration at ball above ground: ", np.around(q_bag,4))




for i in range(100):
    q= robot.shootRandomConfig()
    res, q_sol, err = graph.applyNodeConstraints('ball-above-ground',q)
    if res: break
print(q_sol)

q2 = q1 [::]
q2 [7] = .2

res, q_goal, error = graph.applyNodeConstraints ('placement', q2)


# path planning solver
# ps.setInitialConfig (q_init)
# ps.addGoalConfig (q_goal)
# ps.solve()

################ Viewer display #############################
v = vf.createViewer ()
pp = PathPlayer (v)
# pp(0)
