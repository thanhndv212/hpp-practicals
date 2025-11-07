"""
Manipulation planning script for grasping a ball from an open box and placing it on a table.

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

from math import sqrt
import numpy as np
from hpp import Transform
from hpp.corbaserver.manipulation import ConstraintGraph, Constraints
from hpp.corbaserver import Client

# Reset problem BEFORE importing manipulation module
Client().problem.resetProblem()

from manipulation import (
    robot,
    vf,
    ps,
    Ground,
    Box,
    Pokeball,
    PathPlayer,
    gripperName,
    ballName,
)

# ============================================================================
# Configuration Parameters
# ============================================================================


class Config:
    """Configuration constants for the manipulation task."""

    # Box position
    BOX_X = 0.3
    BOX_OFFSET = 0.04

    # Ball configuration
    BALL_RADIUS = 0.025
    BALL_IN_GRIPPER = [0, 0.137, 0, 0.5, 0.5, -0.5, 0.5]
    BALL_ON_TABLE = [0, 0, BALL_RADIUS, 0, 0, 0, 1]
    BALL_NEAR_TABLE = [BOX_X, 0, 0.1, 0, 0, 0, 1]

    # Gripper-ball alignment
    GRIPPER_BALL_OFFSET = [0, 0.2, 0, 0.5, 0.5, -0.5, 0.5]

    # Initial configuration
    Q_INIT = [0, -1.57, 1.57, 0, 0, 0, 0.3, 0, 0.025, 0, 0, 0, 1]

    # Path planning parameters
    PATH_VALIDATION_STEP = 0.01
    PATH_PROJECTOR_STEP = 0.1
    MAX_RANDOM_ATTEMPTS = 100


# ============================================================================
# Environment Setup
# ============================================================================


def setup_environment():
    """Load and configure the environment objects."""

    # Load ground
    vf.loadEnvironmentModel(Ground, "ground")

    # Load and position box walls
    vf.loadEnvironmentModel(Box, "box")
    box_positions = [
        (
            "box/base_link_0",
            [Config.BOX_X + Config.BOX_OFFSET, 0, Config.BOX_OFFSET],
        ),
        (
            "box/base_link_1",
            [Config.BOX_X - Config.BOX_OFFSET, 0, Config.BOX_OFFSET],
        ),
        (
            "box/base_link_2",
            [Config.BOX_X, Config.BOX_OFFSET, Config.BOX_OFFSET],
        ),
        (
            "box/base_link_3",
            [Config.BOX_X, -Config.BOX_OFFSET, Config.BOX_OFFSET],
        ),
    ]

    for name, pos in box_positions:
        vf.moveObstacle(name, pos + [0, 0, 0, 1])

    # Load ball and set joint bounds
    vf.loadObjectModel(Pokeball, "pokeball")
    robot.setJointBounds(
        "pokeball/root_joint",
        [
            -0.4,
            0.4,
            -0.4,
            0.4,
            -0.1,
            1.0,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
            -1.0001,
            1.0001,
        ],
    )


# ============================================================================
# Constraint Graph Construction
# ============================================================================


def create_constraint_graph():
    """Create and configure the constraint graph for manipulation."""
    graph = ConstraintGraph(robot, "graph")

    # Create nodes - must be created all at once
    nodes = [
        "placement",
        "gripper-above-ball",
        "grasp-placement",
        "ball-above-ground",
        "grasp",
    ]
    graph.createNode(nodes)

    return graph


def create_transformation_constraints():
    """Define all transformation constraints for the task."""

    # Grasp constraint: gripper holds ball
    ps.createTransformationConstraint(
        "grasp", gripperName, ballName, Config.BALL_IN_GRIPPER, 6 * [True]
    )

    # Placement constraint: ball on table (fixed z, roll, pitch)
    ps.createTransformationConstraint(
        "placement",
        "",
        ballName,
        Config.BALL_ON_TABLE,
        [False, False, True, True, True, False],
    )

    # Placement complement: ball can move in x-y, rotate in yaw
    ps.createTransformationConstraint(
        "placement/complement",
        "",
        ballName,
        Config.BALL_ON_TABLE,
        [True, True, False, False, False, True],
    )

    # Gripper-ball alignment: gripper above ball
    ps.createTransformationConstraint(
        "gripper_ball_aligned",
        gripperName,
        ballName,
        Config.GRIPPER_BALL_OFFSET,
        [True, True, True, True, True, True],
    )

    # Ball near table constraint
    ps.createTransformationConstraint(
        "ball_near_table",
        "",
        ballName,
        Config.BALL_NEAR_TABLE,
        [False, False, True, True, True, False],
    )

    # Ball near table complement
    ps.createTransformationConstraint(
        "ball_near_table/complement",
        "",
        ballName,
        [Config.BOX_X, 0.2, 0.1, 0, 0, 0, 1],
        [True, True, False, False, False, True],
    )


def setup_graph_edges(graph):
    """Create edges and assign constraints to the graph."""

    # Define edge configuration: (from_node, to_node, edge_name, weight, grasp_state, constraints)
    edges = [
        # Within placement state
        (
            "placement",
            "placement",
            "transit",
            1,
            "placement",
            ["placement/complement"],
        ),
        # Approach and move away from ball
        (
            "placement",
            "gripper-above-ball",
            "approach-ball",
            1,
            "placement",
            ["placement/complement"],
        ),
        (
            "gripper-above-ball",
            "placement",
            "move-gripper-away",
            1,
            "placement",
            ["placement/complement"],
        ),
        # Grasp and release ball
        (
            "gripper-above-ball",
            "grasp-placement",
            "grasp-ball",
            1,
            "placement",
            ["placement/complement"],
        ),
        (
            "grasp-placement",
            "gripper-above-ball",
            "move-gripper-up",
            1,
            "placement",
            ["placement/complement"],
        ),
        # Lift and place ball
        (
            "grasp-placement",
            "ball-above-ground",
            "take-ball-up",
            1,
            "grasp",
            ["ball_near_table/complement"],
        ),
        (
            "ball-above-ground",
            "grasp-placement",
            "put-ball-down",
            1,
            "grasp",
            ["ball_near_table/complement"],
        ),
        # Move ball away from table
        ("ball-above-ground", "grasp", "take-ball-away", 1, "grasp", []),
        ("grasp", "ball-above-ground", "approach-ground", 1, "grasp", []),
        # Within grasp state
        ("grasp", "grasp", "transfer", 1, "grasp", []),
    ]

    for (
        from_node,
        to_node,
        edge_name,
        weight,
        grasp_state,
        constraints,
    ) in edges:
        graph.createEdge(from_node, to_node, edge_name, weight, grasp_state)
        graph.addConstraints(
            edge=edge_name, constraints=Constraints(numConstraints=constraints)
        )


def setup_graph_nodes(graph):
    """Assign constraints to graph nodes."""

    # Node constraints: (node_name, constraints)
    node_constraints = [
        ("placement", ["placement"]),
        ("gripper-above-ball", ["placement", "gripper_ball_aligned"]),
        ("grasp-placement", ["grasp", "placement"]),
        ("ball-above-ground", ["grasp", "ball_near_table"]),
        ("grasp", ["grasp"]),
    ]

    for node, constraints in node_constraints:
        graph.addConstraints(
            node=node, constraints=Constraints(numConstraints=constraints)
        )


def configure_problem_solver():
    """Configure the path planning solver."""
    ps.selectPathValidation("Discretized", Config.PATH_VALIDATION_STEP)
    ps.selectPathProjector("Progressive", Config.PATH_PROJECTOR_STEP)

    # Set constant right-hand side for constraints
    ps.setConstantRightHandSide("placement", True)
    ps.setConstantRightHandSide("placement/complement", False)
    ps.setConstantRightHandSide("ball_near_table/complement", False)


# ============================================================================
# Configuration Generation
# ============================================================================


def generate_target_config(graph, edge_name, q_from, max_attempts=None):
    """
    Generate a target configuration for a given edge.

    Args:
        graph: Constraint graph
        edge_name: Name of the edge
        q_from: Starting configuration
        max_attempts: Maximum random attempts (default: Config.MAX_RANDOM_ATTEMPTS)

    Returns:
        tuple: (success, configuration, error)
    """
    if max_attempts is None:
        max_attempts = Config.MAX_RANDOM_ATTEMPTS

    for _ in range(max_attempts):
        q_rand = robot.shootRandomConfig()
        res, q_target, err = graph.generateTargetConfig(
            edge_name, q_from, q_rand
        )
        if res:
            return res, q_target, err

    raise RuntimeError(
        f"Failed to generate target config for edge '{edge_name}' after {max_attempts} attempts"
    )


def apply_node_constraint(graph, node_name, q):
    """
    Apply node constraints to a configuration.

    Args:
        graph: Constraint graph
        node_name: Name of the node
        q: Configuration

    Returns:
        tuple: (success, configuration, error)
    """
    res, q_constrained, err = graph.applyNodeConstraints(node_name, q)
    if not res:
        raise RuntimeError(
            f"Failed to apply constraints for node '{node_name}': {err}"
        )
    return res, q_constrained, err


def generate_configurations(graph):
    """
    Generate all intermediate configurations for the manipulation task.

    Returns:
        list: List of configurations
    """
    Q = []

    print("Generating configurations for manipulation task...")

    # 1. Initial placement
    _, q_init, _ = apply_node_constraint(graph, "placement", Config.Q_INIT)
    print(f"1. Initial placement: {np.around(q_init, 4)}")
    Q.append(q_init)

    # 2. Approach ball
    _, q_ab, _ = generate_target_config(graph, "approach-ball", q_init)
    print(f"2. After approach-ball: {np.around(q_ab, 4)}")
    Q.append(q_ab)

    # 3. Gripper above ball
    _, q_gab, _ = apply_node_constraint(graph, "gripper-above-ball", q_ab)
    print(f"3. Gripper above ball: {np.around(q_gab, 4)}")
    Q.append(q_gab)

    # 4. Grasp ball
    _, q_gb, _ = generate_target_config(graph, "grasp-ball", q_gab)
    print(f"4. After grasp ball: {np.around(q_gb, 4)}")
    Q.append(q_gb)

    # 5. Grasp placement
    _, q_gp, _ = apply_node_constraint(graph, "grasp-placement", q_gb)
    print(f"5. Grasp placement: {np.around(q_gp, 4)}")
    Q.append(q_gp)

    # 6. Take ball up
    _, q_tbu, _ = generate_target_config(graph, "take-ball-up", q_gp)
    print(f"6. After take ball up: {np.around(q_tbu, 4)}")
    Q.append(q_tbu)

    # 7. Ball above ground
    _, q_bag, _ = apply_node_constraint(graph, "ball-above-ground", q_tbu)
    print(f"7. Ball above ground: {np.around(q_bag, 4)}")
    Q.append(q_bag)

    # 8. Take ball away
    _, q_tba, _ = generate_target_config(graph, "take-ball-away", q_bag)
    print(f"8. After take ball away: {np.around(q_tba, 4)}")
    Q.append(q_tba)

    # 9. Grasp state
    _, q_g, _ = apply_node_constraint(graph, "grasp", q_tba)
    print(f"9. Grasp state: {np.around(q_g, 4)}")
    Q.append(q_g)

    # 10. Approach ground (with modified goal)
    q_goal_temp = Config.Q_INIT[::]
    q_goal_temp[7] = 0.2
    _, q_ag, _ = graph.generateTargetConfig(
        "approach-ground", q_g, q_goal_temp
    )
    print(f"10. After approach ground: {np.around(q_ag, 4)}")
    Q.append(q_ag)

    # 11. Ball above ground (return)
    _, q_bag2, _ = apply_node_constraint(graph, "ball-above-ground", q_ag)
    print(f"11. Ball above ground (return): {np.around(q_bag2, 4)}")
    Q.append(q_bag2)

    # 12. Put ball down
    _, q_pbd, _ = generate_target_config(graph, "put-ball-down", q_bag)
    print(f"12. After put ball down: {np.around(q_pbd, 4)}")
    Q.append(q_pbd)

    # 13. Grasp placement (release)
    _, q_gp2, _ = apply_node_constraint(graph, "grasp-placement", q_pbd)
    print(f"13. Grasp placement (release): {np.around(q_gp2, 4)}")
    Q.append(q_gp2)

    # 14. Move gripper up
    _, q_mgu, _ = generate_target_config(graph, "move-gripper-up", q_gp)
    print(f"14. After move gripper up: {np.around(q_mgu, 4)}")
    Q.append(q_mgu)

    # 15. Gripper above ball (final)
    _, q_gab2, _ = apply_node_constraint(graph, "gripper-above-ball", q_mgu)
    print(f"15. Gripper above ball (final): {np.around(q_gab2, 4)}")
    Q.append(q_gab2)

    # 16. Move gripper away
    _, q_mga, _ = generate_target_config(graph, "move-gripper-away", q_gab)
    print(f"16. After move gripper away: {np.around(q_mga, 4)}")
    Q.append(q_mga)

    return Q


# ============================================================================
# Path Planning
# ============================================================================


def solve_manipulation_problem(graph):
    """
    Solve the complete manipulation planning problem.

    Args:
        graph: Constraint graph

    Returns:
        tuple: (initial_config, goal_config)
    """
    # Set initial configuration
    _, q_init, _ = apply_node_constraint(graph, "placement", Config.Q_INIT)

    # Set goal configuration (ball at different x position)
    q_goal_base = Config.Q_INIT[::]
    q_goal_base[7] = 0.2  # Move ball in x direction
    _, q_goal, _ = apply_node_constraint(graph, "placement", q_goal_base)

    # Configure and solve
    ps.setInitialConfig(q_init)
    ps.addGoalConfig(q_goal)

    print("Solving manipulation problem...")
    ps.solve()
    print("Solution found!")

    return q_init, q_goal


# ============================================================================
# Main Execution
# ============================================================================


def main():
    """Main execution function."""

    # Setup environment
    print("Setting up environment...")
    setup_environment()

    # Create constraint graph
    print("Creating constraint graph...")
    graph = create_constraint_graph()

    # Define constraints
    print("Defining transformation constraints...")
    create_transformation_constraints()

    # Setup graph structure
    print("Setting up graph edges and nodes...")
    setup_graph_edges(graph)
    setup_graph_nodes(graph)

    # Configure solver
    print("Configuring problem solver...")
    configure_problem_solver()

    # Initialize graph
    print("Initializing constraint graph...")
    graph.initialize()

    # Generate intermediate configurations (for debugging/visualization)
    print("\n" + "=" * 60)
    print("Generating intermediate configurations...")
    print("=" * 60)
    Q = generate_configurations(graph)

    # Solve the full problem
    print("\n" + "=" * 60)
    print("Solving complete manipulation problem...")
    print("=" * 60)
    q_init, q_goal = solve_manipulation_problem(graph)

    # Visualization
    print("\n" + "=" * 60)
    print("Starting visualization...")
    print("=" * 60)
    v = vf.createViewer()
    pp = PathPlayer(v)
    pp(0)

    print("\nDone! Use PathPlayer to view the solution.")
    return graph, Q, v, pp


# ============================================================================
# Execute
# ============================================================================

if __name__ == "__main__":
    graph, configurations, viewer, path_player = main()
