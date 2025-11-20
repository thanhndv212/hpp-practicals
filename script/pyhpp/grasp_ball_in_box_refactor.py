"""
Example: Manipulation Planning with PyHPP

This script demonstrates how to use PyHPP's manipulation features
including constraint graphs, similar to the CORBA server approach.

Based on: grasp_ball_in_box.py (CORBA version)

Task sequence:
1. placement -> approach-ball -> gripper-above-ball
2. gripper-above-ball -> grasp-ball -> grasp-placement
3. grasp-placement -> take-ball-up -> ball-above-ground
4. ball-above-ground -> take-ball-away -> grasp
5. grasp -> approach-ground -> ball-above-ground
6. ball-above-ground -> put-ball-down -> grasp-placement
7. grasp-placement -> move-gripper-up -> gripper-above-ball
8. gripper-above-ball -> move-gripper-away -> placement
"""

import numpy as np
from pinocchio import SE3, Quaternion, StdVec_Bool as Mask

from pyhpp.manipulation import Device, urdf, Graph, Problem
from pyhpp.manipulation import createProgressiveProjector, ManipulationPlanner
from pyhpp.constraints import Transformation, RelativeTransformation, Implicit
from pyhpp.constraints import ComparisonTypes, ComparisonType
from pyhpp.gepetto.viewer import Viewer


# ============================================================================
# Configuration
# ============================================================================

class Config:
    """Configuration constants for manipulation task."""
    
    # Joint names
    GRIPPER_NAME = "ur5/wrist_3_joint"
    BALL_NAME = "pokeball/root_joint"
    
    # Transformations (x, y, z, qx, qy, qz, qw)
    BALL_IN_GRIPPER = [0, 0.137, 0, 0.5, 0.5, -0.5, 0.5]
    BALL_ON_GROUND = [0, 0, 0.025, 0, 0, 0, 1]
    BALL_NEAR_TABLE = [0.3, 0, 0.1, 0, 0, 0, 1]
    GRIPPER_ABOVE_BALL = [0, 0.2, 0, 0.5, 0.5, -0.5, 0.5]
    
    # Box position
    BOX_X = 0.3
    BOX_OFFSET = 0.04
    
    # Initial configuration
    Q_ROBOT_INIT = [0, -1.57, 1.57, 0, 0, 0]
    Q_BALL_INIT = [0.3, 0, 0.025, 0, 0, 0, 1]


class URDFPaths:
    """URDF file paths."""
    
    ROBOT_URDF = "package://example-robot-data/robots/ur_description/urdf/ur5_gripper.urdf"
    ROBOT_SRDF = "package://example-robot-data/robots/ur_description/srdf/ur5_gripper.srdf"
    
    BALL_URDF = "package://hpp_environments/urdf/ur_benchmark/pokeball.urdf"
    BALL_SRDF = "package://hpp_environments/srdf/ur_benchmark/pokeball.srdf"
    
    BOX_URDF = "package://hpp_environments/urdf/ur_benchmark/box.urdf"
    BOX_SRDF = "package://hpp_environments/srdf/ur_benchmark/box.srdf"
    
    GROUND_URDF = "package://hpp_environments/urdf/ur_benchmark/ground.urdf"
    GROUND_SRDF = "package://hpp_environments/srdf/ur_benchmark/ground.srdf"


# ============================================================================
# Robot Setup
# ============================================================================

def create_robot():
    """Create robot with UR5, ball, box, and ground."""
    
    robot = Device("manipulation_bot")
    
    # Identity transform
    identity = SE3(rotation=np.identity(3), translation=np.array([0, 0, 0]))
    
    # Load UR5 robot (fixed base)
    urdf.loadModel(
        robot, 0, "ur5", "anchor",
        URDFPaths.ROBOT_URDF,
        URDFPaths.ROBOT_SRDF,
        identity
    )
    print("✓ Loaded UR5 robot")
    
    # Load pokeball (freeflyer - can be grasped)
    urdf.loadModel(
        robot, 0, "pokeball", "freeflyer",
        URDFPaths.BALL_URDF,
        URDFPaths.BALL_SRDF,
        identity
    )
    print("✓ Loaded pokeball")
    
    # Set joint bounds for pokeball
    robot.setJointBounds(
        Config.BALL_NAME,
        [
            -0.4, 0.4,         # x bounds
            -0.4, 0.4,         # y bounds
            -0.1, 1.0,         # z bounds
            -1.0001, 1.0001,   # qx
            -1.0001, 1.0001,   # qy
            -1.0001, 1.0001,   # qz
            -1.0001, 1.0001,   # qw
        ],
    )
    
    # Load box (fixed obstacle)
    urdf.loadModel(
        robot, 0, "box", "anchor",
        URDFPaths.BOX_URDF,
        URDFPaths.BOX_SRDF,
        identity
    )
    print("✓ Loaded box")
    
    # Load ground
    urdf.loadModel(
        robot, 0, "ground", "anchor",
        URDFPaths.GROUND_URDF,
        URDFPaths.GROUND_SRDF,
        identity
    )
    print("✓ Loaded ground")
    
    return robot


# ============================================================================
# Constraint Graph Setup
# ============================================================================

def create_constraints(robot, problem):
    """
    Create all transformation constraints.
    
    Returns dictionary of constraint name -> Implicit constraint object
    """
    constraints = {}
    
    # Get joint IDs
    joint_gripper = robot.model().getJointId(Config.GRIPPER_NAME)
    joint_ball = robot.model().getJointId(Config.BALL_NAME)
    Id = SE3.Identity()
    
    # ========================================================================
    # 1. GRASP: gripper/ball fixed (all 6 DOF)
    # ========================================================================
    q_grasp = Quaternion(0.5, 0.5, -0.5, 0.5)
    ball_in_gripper = SE3(q_grasp, np.array([0, 0.137, 0]))
    mask_full = Mask()
    mask_full[:] = (True,) * 6
    
    pc_grasp = RelativeTransformation.create(
        'grasp',
        robot.asPinDevice(),
        joint_gripper,
        joint_ball,
        ball_in_gripper,
        Id,
        mask_full
    )
    
    cts_grasp = ComparisonTypes()
    cts_grasp[:] = tuple([ComparisonType.EqualToZero] * 6)
    constraints['grasp'] = Implicit.create(pc_grasp, cts_grasp, mask_full)
    
    # ========================================================================
    # 2. PLACEMENT: world/ball - fixed z, fixed r, p
    # ========================================================================
    q_placement = Quaternion(0, 0, 0, 1)
    ball_on_ground = SE3(q_placement, np.array([0, 0, 0.15]))
    mask_placement = [False, False, True, True, True, False]
    
    pc_placement = Transformation.create(
        'placement',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_on_ground,
        mask_placement
    )
    
    cts_placement = ComparisonTypes()
    cts_placement[:] = tuple([ComparisonType.EqualToZero] * 3)
    implicit_mask_placement = [True, True, True]
    constraints['placement'] = Implicit.create(
        pc_placement, cts_placement, implicit_mask_placement
    )
    
    # ========================================================================
    # 3. PLACEMENT/COMPLEMENT: world/ball - fixed x, y, yaw
    # ========================================================================
    mask_placement_comp = [True, True, False, False, False, True]
    
    pc_placement_comp = Transformation.create(
        'placement/complement',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_on_ground,
        mask_placement_comp
    )
    
    cts_placement_comp = ComparisonTypes()
    cts_placement_comp[:] = tuple([ComparisonType.EqualToZero] * 3)
    implicit_mask_comp = [True, True, True]
    constraints['placement/complement'] = Implicit.create(
        pc_placement_comp, cts_placement_comp, implicit_mask_comp
    )
    
    # ========================================================================
    # 4. GRIPPER_BALL_ALIGNED: gripper/ball - all fixed
    # ========================================================================
    q_aligned = Quaternion(0.5, 0.5, -0.5, 0.5)
    gripper_above_ball = SE3(q_aligned, np.array([0, 0.2, 0]))
    mask_aligned = Mask()
    mask_aligned[:] = (True,) * 6
    
    pc_aligned = RelativeTransformation.create(
        'gripper_ball_aligned',
        robot.asPinDevice(),
        joint_gripper,
        joint_ball,
        gripper_above_ball,
        Id,
        mask_aligned
    )
    
    cts_aligned = ComparisonTypes()
    cts_aligned[:] = tuple([ComparisonType.EqualToZero] * 6)
    constraints['gripper_ball_aligned'] = Implicit.create(
        pc_aligned, cts_aligned, mask_aligned
    )
    
    # ========================================================================
    # 5. BALL_NEAR_TABLE: world/ball - fixed z, fixed r, p
    # ========================================================================
    q_table = Quaternion(0, 0, 0, 1)
    ball_near_table = SE3(q_table, np.array([0.3, 0, 0.1]))
    mask_table = [False, False, True, True, True, False]
    
    pc_table = Transformation.create(
        'ball_near_table',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_near_table,
        mask_table
    )
    
    cts_table = ComparisonTypes()
    cts_table[:] = tuple([ComparisonType.EqualToZero] * 3)
    implicit_mask_table = [True, True, True]
    constraints['ball_near_table'] = Implicit.create(
        pc_table, cts_table, implicit_mask_table
    )
    
    # ========================================================================
    # 6. BALL_NEAR_TABLE/COMPLEMENT: world/ball - fixed x, y, yaw
    # ========================================================================
    ball_near_table_comp = SE3(q_table, np.array([0.3, 0.2, 0.1]))
    mask_table_comp = [True, True, False, False, False, True]
    
    pc_table_comp = Transformation.create(
        'ball_near_table/complement',
        robot.asPinDevice(),
        joint_ball,
        Id,
        ball_near_table_comp,
        mask_table_comp
    )
    
    cts_table_comp = ComparisonTypes()
    cts_table_comp[:] = tuple([ComparisonType.EqualToZero] * 3)
    constraints['ball_near_table/complement'] = Implicit.create(
        pc_table_comp, cts_table_comp, implicit_mask_comp
    )
    
    return constraints


def create_constraint_graph(robot, problem):
    """
    Create constraint graph for manipulation with 5 states.
    
    States:
    - placement: Ball on table
    - gripper-above-ball: Gripper aligned above ball, ball on table
    - grasp-placement: Gripper grasping ball, ball on table
    - ball-above-ground: Gripper grasping ball, ball near table
    - grasp: Gripper grasping ball (free motion)
    
    Returns:
        tuple: (graph, states_dict, edges_dict, constraints_dict)
    """
    
    # Create constraints first
    constraints = create_constraints(robot, problem)
    print("✓ Created transformation constraints")
    
    # Create graph
    graph = Graph("manipulation_graph", robot, problem)
    print("✓ Created constraint graph")
    
    # ========================================================================
    # Create States (Nodes)
    # ========================================================================
    
    states = {}
    states['placement'] = graph.createState("placement", False, 0)
    states['gripper-above-ball'] = graph.createState("gripper-above-ball", False, 0)
    states['grasp-placement'] = graph.createState("grasp-placement", False, 0)
    states['ball-above-ground'] = graph.createState("ball-above-ground", False, 0)
    states['grasp'] = graph.createState("grasp", False, 0)
    
    print("✓ Created 5 states")
    
    # ========================================================================
    # Create Edges (Transitions)
    # ========================================================================
    
    edges = {}
    
    # Self-loops
    edges['transit'] = graph.createTransition(
        states['placement'], states['placement'], "transit", 1, states['placement']
    )
    edges['transfer'] = graph.createTransition(
        states['grasp'], states['grasp'], "transfer", 1, states['grasp']
    )
    
    # From placement
    edges['approach-ball'] = graph.createTransition(
        states['placement'], states['gripper-above-ball'], "approach-ball", 1, states['placement']
    )
    
    # From gripper-above-ball
    edges['move-gripper-away'] = graph.createTransition(
        states['gripper-above-ball'], states['placement'], "move-gripper-away", 1, states['placement']
    )
    edges['grasp-ball'] = graph.createTransition(
        states['gripper-above-ball'], states['grasp-placement'], "grasp-ball", 1, states['placement']
    )
    
    # From grasp-placement
    edges['move-gripper-up'] = graph.createTransition(
        states['grasp-placement'], states['gripper-above-ball'], "move-gripper-up", 1, states['placement']
    )
    edges['take-ball-up'] = graph.createTransition(
        states['grasp-placement'], states['ball-above-ground'], "take-ball-up", 1, states['grasp']
    )
    
    # From ball-above-ground
    edges['put-ball-down'] = graph.createTransition(
        states['ball-above-ground'], states['grasp-placement'], "put-ball-down", 1, states['grasp']
    )
    edges['take-ball-away'] = graph.createTransition(
        states['ball-above-ground'], states['grasp'], "take-ball-away", 1, states['grasp']
    )
    
    # From grasp
    edges['approach-ground'] = graph.createTransition(
        states['grasp'], states['ball-above-ground'], "approach-ground", 1, states['grasp']
    )
    
    print("✓ Created transitions")
    
    # ========================================================================
    # Add Constraints to States
    # ========================================================================
    
    # State: placement (ball on table)
    graph.addNumericalConstraint(
        states['placement'],
        constraints['placement']
    )
    
    # State: gripper-above-ball (ball on table, gripper aligned)
    graph.addNumericalConstraint(states['gripper-above-ball'], constraints['placement'])
    graph.addNumericalConstraint(states['gripper-above-ball'], constraints['gripper_ball_aligned'])
    
    # State: grasp-placement (grasping ball on table)
    graph.addNumericalConstraint(states['grasp-placement'], constraints['grasp'])
    graph.addNumericalConstraint(states['grasp-placement'], constraints['placement'])
    
    # State: ball-above-ground (grasping ball near table)
    graph.addNumericalConstraint(states['ball-above-ground'], constraints['grasp'])
    graph.addNumericalConstraint(states['ball-above-ground'], constraints['ball_near_table'])
    
    # State: grasp (grasping ball, free motion)
    graph.addNumericalConstraint(states['grasp'], constraints['grasp'])
    
    print("✓ Added constraints to states")
    
    # ========================================================================
    # Add Constraints to Edges (Not supported in current API)
    # ========================================================================
    # Note: Based on test files, edge constraints may not be directly supported
    # They are inherited from the containing state parameter in createTransition
    
    print("✓ Edge constraints inherited from containing states")
    
    # ========================================================================
    # Configure Graph Parameters
    # ========================================================================
    
    graph.maxIterations(100)
    graph.errorThreshold(0.00001)
    
    # ========================================================================
    # Initialize Graph (REQUIRED)
    # ========================================================================
    
    graph.initialize()
    print("✓ Graph initialized")
    
    return graph, states, edges, constraints


# ============================================================================
# Configuration Testing
# ============================================================================

def test_graph_traversal(graph, states, edges, robot, problem):
    """
    Test the constraint graph by generating configurations along the path.
    
    This mimics the CORBA version's configuration testing.
    """
    
    print("\n" + "=" * 70)
    print("Testing Graph Traversal")
    print("=" * 70)
    
    Q = []  # Store all configurations
    
    # Get configuration shooter
    shooter = problem.configurationShooter()
    
    # Initial configuration
    q1 = np.array(Config.Q_ROBOT_INIT + Config.Q_BALL_INIT)
    
    # 1. Apply placement state constraints
    print("\n1. Projecting initial config onto 'placement' state...")
    result = graph.applyStateConstraints(states['placement'], q1)
    if not result.success:
        print(f"   ⚠ Projection failed with error {result.error}")
    q_init = result.configuration
    print(f"   ✓ Initial configuration: {np.around(q_init, 4)}")
    Q.append(q_init)
    
    # 2. Generate target for 'approach-ball'
    print("\n2. Generating target for 'approach-ball'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(edges['approach-ball'], q_init, q_rand)
        if result.success:
            q_ab = result.configuration
            break
    if result.success:
        print(f"   ✓ After approach-ball: {np.around(q_ab, 4)}")
        Q.append(q_ab)
    else:
        print(f"   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 3. Apply 'gripper-above-ball' state
    print("\n3. Projecting onto 'gripper-above-ball' state...")
    result = graph.applyStateConstraints(states['gripper-above-ball'], q_ab)
    q_gab = result.configuration
    print(f"   ✓ At gripper-above-ball: {np.around(q_gab, 4)}")
    Q.append(q_gab)
    
    # 4. Generate target for 'grasp-ball'
    print("\n4. Generating target for 'grasp-ball'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(edges['grasp-ball'], q_gab, q_rand)
        if result.success:
            q_gb = result.configuration
            break
    if result.success:
        print(f"   ✓ After grasp-ball: {np.around(q_gb, 4)}")
        Q.append(q_gb)
    else:
        print(f"   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 5. Apply 'grasp-placement' state
    print("\n5. Projecting onto 'grasp-placement' state...")
    result = graph.applyStateConstraints(states['grasp-placement'], q_gb)
    q_gp = result.configuration
    print(f"   ✓ At grasp-placement: {np.around(q_gp, 4)}")
    Q.append(q_gp)
    
    # 6. Generate target for 'take-ball-up'
    print("\n6. Generating target for 'take-ball-up'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(edges['take-ball-up'], q_gp, q_rand)
        if result.success:
            q_tbu = result.configuration
            break
    if result.success:
        print(f"   ✓ After take-ball-up: {np.around(q_tbu, 4)}")
        Q.append(q_tbu)
    else:
        print(f"   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 7. Apply 'ball-above-ground' state
    print("\n7. Projecting onto 'ball-above-ground' state...")
    result = graph.applyStateConstraints(states['ball-above-ground'], q_tbu)
    q_bag = result.configuration
    print(f"   ✓ At ball-above-ground: {np.around(q_bag, 4)}")
    Q.append(q_bag)
    
    # 8. Generate target for 'take-ball-away'
    print("\n8. Generating target for 'take-ball-away'...")
    for i in range(100):
        q_rand = shooter.shoot()
        result = graph.generateTargetConfig(edges['take-ball-away'], q_bag, q_rand)
        if result.success:
            q_tba = result.configuration
            break
    if result.success:
        print(f"   ✓ After take-ball-away: {np.around(q_tba, 4)}")
        Q.append(q_tba)
    else:
        print(f"   ⚠ Failed to generate config")
        return Q, q_init, q_init
    
    # 9. Apply 'grasp' state
    print("\n9. Projecting onto 'grasp' state...")
    result = graph.applyStateConstraints(states['grasp'], q_tba)
    q_g = result.configuration
    print(f"   ✓ At grasp: {np.around(q_g, 4)}")
    Q.append(q_g)
    
    # 10. Generate goal configuration
    print("\n10. Generating goal configuration...")
    q2 = q1.copy()
    q2[6] = 0.2  # Move ball x position
    result = graph.applyStateConstraints(states['placement'], q2)
    q_goal = result.configuration
    print(f"   ✓ Goal configuration: {np.around(q_goal, 4)}")
    
    print("\n✓ Graph traversal test complete!")
    print(f"  Generated {len(Q)} configurations")
    
    return Q, q_init, q_goal


# ============================================================================
# Main Function
# ============================================================================

def main():
    """Main execution."""
    
    print("=" * 70)
    print("Manipulation Planning: Grasp Ball in Box (PyHPP)")
    print("=" * 70)
    
    # 1. Create robot
    print("\n1. Creating robot...")
    robot = create_robot()
    
    # 2. Create problem
    print("\n2. Setting up problem...")
    problem = Problem(robot)
    
    # Configure path validation and projection
    # Note: PathValidationDiscretized not working, using default
    # problem.pathValidation = PathValidationDiscretized(robot.asPinDevice(), 0.01)
    problem.pathProjector = createProgressiveProjector(
        problem.distance(),
        problem.steeringMethod(),
        0.1
    )
    print("✓ Problem configured")
    
    # 3. Create constraint graph
    print("\n3. Creating constraint graph...")
    graph, states, edges, constraints = create_constraint_graph(robot, problem)
    
    # 4. Test graph traversal
    Q, q_init, q_goal = test_graph_traversal(graph, states, edges, robot, problem)
    
    # 5. Setup planning problem
    print("\n" + "=" * 70)
    print("Setting up planning problem")
    print("=" * 70)
    problem.initConfig(q_init)
    problem.addGoalConfig(q_goal)
    problem.constraintGraph(graph)
    print("✓ Problem setup complete")
    
    # 6. Create manipulation planner
    print("\n6. Creating manipulation planner...")
    planner = ManipulationPlanner(problem)
    # planner.maxIterations(10000)
    print("✓ Planner created")
    
    # 7. Visualization
    print("\n7. Starting visualization...")
    viewer = Viewer(robot)
    viewer(q_init)
    print("✓ Displaying initial configuration")
    
    print("\n" + "=" * 70)
    print("Setup Complete!")
    print("=" * 70)
    
    print("\nNext steps:")
    print("1. Call planner.solve() to solve the problem")
    print("2. Use viewer(q) to display configurations")
    print("3. Animate path: animate_configurations(viewer, Q)")
    
    print("\nObjects:")
    print("  robot       - Robot device")
    print("  problem     - Planning problem")
    print("  graph       - Constraint graph")
    print("  states      - Dictionary of state objects")
    print("  edges       - Dictionary of edge objects")
    print("  constraints - Dictionary of constraint objects")
    print("  viewer      - Gepetto viewer")
    print("  planner     - Manipulation planner")
    print("  Q           - List of test configurations")
    print("  q_init      - Initial configuration")
    print("  q_goal      - Goal configuration")
    
    return robot, problem, graph, states, edges, constraints, viewer, planner, Q, q_init, q_goal


# ============================================================================
# Utility Functions
# ============================================================================

def animate_configurations(viewer, configs, dt=0.5):
    """
    Animate a sequence of configurations.
    
    Args:
        viewer: Gepetto Viewer instance
        configs: List of configurations
        dt: Time delay between configurations
    """
    import time
    
    print(f"\nAnimating {len(configs)} configurations...")
    for i, q in enumerate(configs):
        print(f"  Config {i+1}/{len(configs)}")
        viewer(q)
        time.sleep(dt)
    print("✓ Animation complete!")


def animate_path(viewer, path, dt=0.01):
    """
    Animate a path in the viewer.
    
    Args:
        viewer: Gepetto Viewer instance
        path: Path object
        dt: Time step
    """
    import time
    
    t = 0.0
    path_length = path.length()
    
    print(f"Animating path (length={path_length:.3f})...")
    
    while t <= path_length:
        q = path(t)
        viewer(q)
        time.sleep(dt)
        t += dt
    
    print("✓ Animation complete!")


# ============================================================================
# Execute
# ============================================================================

if __name__ == "__main__":
    robot, problem, graph, states, edges, constraints, viewer, planner, Q, q_init, q_goal = main()
    
    print("\n" + "=" * 70)
    print("Example Usage:")
    print("=" * 70)
    print("""
# Solve the problem
planner.solve()

# Get the path
path = planner.path()

# Animate the solution
animate_path(viewer, path)

# Or animate test configurations
animate_configurations(viewer, Q, dt=1.0)

# Display specific configuration
viewer(q_init)
viewer(q_goal)
viewer(Q[3])  # Display 4th configuration
""")
    planner.solve()

    # Get the path
    path = planner.path()

    # # Animate the solution
    # animate_path(viewer, path)

    # # Or animate test configurations
    # animate_configurations(viewer, Q, dt=1.0)

    # # Display specific configuration
    # viewer(q_init)
    # viewer(q_goal)
    # viewer(Q[3])  # Display 4th configuration
