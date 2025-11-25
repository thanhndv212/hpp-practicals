#!/usr/bin/env python3
"""
Grasp ball in box manipulation example.

This example demonstrates manipulation planning to grasp a ball inside a box.
"""

from math import sqrt
from hpp import Transform
import numpy as np
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client
# Reset problem
Client().problem.resetProblem()
from manipulation import (
    robot, vf, ps, Ground, Box, Pokeball,
    PathPlayer, gripperName, ballName
)

# ============================================================================
# Load Environment and Objects
# ============================================================================

# Load environment for viewer
vf.loadEnvironmentModel(Ground, 'ground')
vf.loadEnvironmentModel(Box, 'box')

# Position box walls
vf.moveObstacle('box/base_link_0', [0.3 + 0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle('box/base_link_1', [0.3 - 0.04, 0, 0.04, 0, 0, 0, 1])
vf.moveObstacle('box/base_link_2', [0.3, 0.04, 0.04, 0, 0, 0, 1])
vf.moveObstacle('box/base_link_3', [0.3, -0.04, 0.04, 0, 0, 0, 1])

# Load ball object
vf.loadObjectModel(Pokeball, 'pokeball')

# Set joint bounds for ball (freeflyer)
robot.setJointBounds(
    'pokeball/root_joint',
    [-.4, .4, -.4, .4, -.1, 1.,
     -1.0001, 1.0001, -1.0001, 1.0001,
     -1.0001, 1.0001, -1.0001, 1.0001]
)

# ============================================================================
# Create Constraint Graph
# ============================================================================

graph = ConstraintGraph(robot, 'graph')

# Create nodes (states)
graph.createNode([
    'grasp',
    'ball-above-ground',
    'grasp-placement',
    'gripper-above-ball',
    'placement'
])

# Create edges (transitions)
# Self-loops
graph.createEdge('placement', 'placement', 'transit', 1, 'placement')
graph.createEdge('grasp', 'grasp', 'transfer', 1, 'grasp')

# From placement
graph.createEdge(
    'placement', 'gripper-above-ball', 'approach-ball', 1, 'placement'
)

# From gripper-above-ball
graph.createEdge(
    'gripper-above-ball', 'placement', 'move-gripper-away', 1, 'placement'
)
graph.createEdge(
    'gripper-above-ball', 'grasp-placement', 'grasp-ball', 1, 'placement'
)

# From grasp-placement
graph.createEdge(
    'grasp-placement', 'gripper-above-ball', 'move-gripper-up', 1, 'placement'
)
graph.createEdge(
    'grasp-placement', 'ball-above-ground', 'take-ball-up', 1, 'grasp'
)

# From ball-above-ground
graph.createEdge(
    'ball-above-ground', 'grasp-placement', 'put-ball-down', 1, 'grasp'
)
graph.createEdge(
    'ball-above-ground', 'grasp', 'take-ball-away', 1, 'grasp'
)

# From grasp
graph.createEdge(
    'grasp', 'ball-above-ground', 'approach-ground', 1, 'grasp'
)


# ============================================================================
# Create Transformation Constraints
# ============================================================================

# Configuration constraints
# --------------------------

# Grasp constraint: gripper holds ball (all DOF fixed)
ballInGripper = [0, .137, 0, 0.5, 0.5, -0.5, 0.5]
ps.createTransformationConstraint(
    'grasp', gripperName, ballName,
    ballInGripper,
    6 * [True]
)

# Placement constraint: ball on ground (z, roll, pitch fixed)
ps.createTransformationConstraint(
    'placement', '', ballName,
    [0, 0, 0.025, 0, 0, 0, 1],
    [False, False, True, True, True, False]
)

# Motion constraints
# ------------------

# Placement complement: ball can move in x-y, rotate in yaw
ps.createTransformationConstraint(
    'placement/complement', '', ballName,
    [0, 0, 0.025, 0, 0, 0, 1],
    [True, True, False, False, False, True]
)

# Gripper-ball alignment: gripper above ball (all DOF fixed)
ps.createTransformationConstraint(
    'gripper_ball_aligned', gripperName, ballName,
    [0, .2, 0, 0.5, 0.5, -0.5, 0.5],
    [True, True, True, True, True, True]
)

# Ball near table: ball at specific height near table
ps.createTransformationConstraint(
    'ball_near_table', '', ballName,
    [0.3, 0, 0.1, 0, 0, 0, 1],
    [False, False, True, True, True, False]
)

# Ball near table complement: ball can move in x-y, rotate in yaw
ps.createTransformationConstraint(
    'ball_near_table/complement', '', ballName,
    [0.3, 0.2, 0.1, 0, 0, 0, 1],
    [True, True, False, False, False, True]
)

# ============================================================================
# Add Constraints to Nodes
# ============================================================================

graph.addConstraints(
    node='placement',
    constraints=Constraints(numConstraints=['placement'])
)

graph.addConstraints(
    node='gripper-above-ball',
    constraints=Constraints(numConstraints=['placement', 'gripper_ball_aligned'])
)

graph.addConstraints(
    node='grasp-placement',
    constraints=Constraints(numConstraints=['grasp', 'placement'])
)

graph.addConstraints(
    node='ball-above-ground',
    constraints=Constraints(numConstraints=['grasp', 'ball_near_table'])
)

graph.addConstraints(
    node='grasp',
    constraints=Constraints(numConstraints=['grasp'])
)

# ============================================================================
# Add Constraints to Edges
# ============================================================================

# Ball on table
graph.addConstraints(
    edge='transit',
    constraints=Constraints(numConstraints=['placement/complement'])
)
graph.addConstraints(
    edge='approach-ball',
    constraints=Constraints(numConstraints=['placement/complement'])
)
graph.addConstraints(
    edge='move-gripper-away',
    constraints=Constraints(numConstraints=['placement/complement'])
)

# Gripper above ball, ball on table
graph.addConstraints(
    edge='grasp-ball',
    constraints=Constraints(numConstraints=['placement/complement'])
)
graph.addConstraints(
    edge='move-gripper-up',
    constraints=Constraints(numConstraints=['placement/complement'])
)

# Gripper grasp ball, ball on table
graph.addConstraints(
    edge='take-ball-up',
    constraints=Constraints(numConstraints=['ball_near_table/complement'])
)
graph.addConstraints(
    edge='put-ball-down',
    constraints=Constraints(numConstraints=['ball_near_table/complement'])
)

# Gripper grasp ball, ball near table
graph.addConstraints(
    edge='take-ball-away',
    constraints=Constraints()
)
graph.addConstraints(
    edge='approach-ground',
    constraints=Constraints()
)

# Gripper grasp ball (free motion)
graph.addConstraints(
    edge='transfer',
    constraints=Constraints()
)

# ============================================================================
# Configure Problem Solver
# ============================================================================

ps.selectPathValidation("Discretized", 0.01)
# ps.selectPathValidation("Dichotomy", 0)

ps.selectPathProjector("Progressive", 0.1)

ps.setConstantRightHandSide('placement', True)
ps.setConstantRightHandSide('placement/complement', False)
ps.setConstantRightHandSide('ball_near_table/complement', False)

# Initialize graph
print("Start constructing constraint graph...")
graph.initialize()

# ============================================================================
# Generate Feasible Configurations
# ============================================================================

Q = []

# Initial configuration
q1 = [0, -1.57, 1.57, 0, 0, 0, .3, 0, 0.025, 0, 0, 0, 1]

# Project initial config on placement node
res, q_init, error = graph.applyNodeConstraints('placement', q1)
print("Initial configuration:", np.around(q_init, 4))
Q.append(q_init)

# Generate configuration after approach-ball
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_ab, err = graph.generateTargetConfig('approach-ball', q1, q)
    if res:
        break
print("Configuration after approach-ball:", np.around(q_ab, 4))
Q.append(q_ab)

# Project onto gripper-above-ball node
res, q_gab, err = graph.applyNodeConstraints('gripper-above-ball', q_ab)
print("Configuration at gripper above ball:", np.around(q_gab, 4))
Q.append(q_gab)

# Generate configuration after grasp-ball
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_gb, err = graph.generateTargetConfig('grasp-ball', q_gab, q)
    if res:
        break
print("Configuration after grasp ball:", np.around(q_gb, 4))
Q.append(q_gb)

# Project onto grasp-placement node
res, q_gp, err = graph.applyNodeConstraints('grasp-placement', q_gb)
print("Configuration at grasp placement:", np.around(q_gp, 4))
Q.append(q_gp)

# Generate configuration after take-ball-up
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_tbu, err = graph.generateTargetConfig('take-ball-up', q_gp, q)
    if res:
        break
print("Configuration after take ball up:", np.around(q_tbu, 4))
Q.append(q_tbu)

# Project onto ball-above-ground node
res, q_bag, err = graph.applyNodeConstraints('ball-above-ground', q_tbu)
print("Configuration at ball above ground:", np.around(q_bag, 4))
Q.append(q_bag)

# Generate configuration after take-ball-away
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_tba, err = graph.generateTargetConfig('take-ball-away', q_bag, q)
    if res:
        break
print("Configuration after take ball away:", np.around(q_tba, 4))
Q.append(q_tba)

# Project onto grasp node
res, q_g, err = graph.applyNodeConstraints('grasp', q_tba)
print("Configuration at grasp:", np.around(q_g, 4))
Q.append(q_g)

# Goal configuration (move ball to x=0.2)
q2 = q1[::]
q2[7] = .2

# Generate configuration after approach-ground
res, q_ag, err = graph.generateTargetConfig('approach-ground', q_g, q2)
print("Configuration after approach ground:", np.around(q_ag, 4))
Q.append(q_ag)

# Project onto ball-above-ground node
res, q_bag_, err = graph.applyNodeConstraints('ball-above-ground', q_ag)
print("Configuration at ball above ground:", np.around(q_bag_, 4))
Q.append(q_bag_)

# Generate configuration after put-ball-down
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_pbd, err = graph.generateTargetConfig('put-ball-down', q_bag, q)
    if res:
        break
print("Configuration after put ball down:", np.around(q_pbd, 4))
Q.append(q_pbd)

# Project onto grasp-placement node
res, q_gp_, err = graph.applyNodeConstraints('grasp-placement', q_pbd)
print("Configuration at grasp placement:", np.around(q_gp_, 4))
Q.append(q_gp_)

# Generate configuration after move-gripper-up
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_mgu, err = graph.generateTargetConfig('move-gripper-up', q_gp, q)
    if res:
        break
print("Configuration after move gripper up:", np.around(q_mgu, 4))
Q.append(q_mgu)

# Project onto gripper-above-ball node
res, q_gab_, err = graph.applyNodeConstraints('gripper-above-ball', q_mgu)
print("Configuration at gripper above ball:", np.around(q_gab_, 4))
Q.append(q_gab_)

# Generate configuration after move-gripper-away
for i in range(100):
    q = robot.shootRandomConfig()
    res, q_mga, err = graph.generateTargetConfig('move-gripper-away', q_gab, q)
    if res:
        break
print("Configuration after move gripper away:", np.around(q_mga, 4))
Q.append(q_mga)

# Final goal: project onto placement with ball at new location
q2 = q1[::]
q2[7] = .2
res, q_goal, error = graph.applyNodeConstraints('placement', q2)

# ============================================================================
# Solve Manipulation Problem
# ============================================================================

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
ps.solve()

# ============================================================================
# Visualization
# ============================================================================

v = vf.createViewer()
pp = PathPlayer(v)

# Play solution path
pp(0)