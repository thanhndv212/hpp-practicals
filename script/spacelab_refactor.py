"""
Spacelab manipulation planning script.

This script sets up a multi-robot manipulation scenario with:
- UR10 and VISPA robots
- Multiple objects: RS1, ScrewDriver, FrameGripper, CleatGripper
- Constraint graph for manipulation planning
"""

import numpy as np
from pinocchio import SE3, SE3ToXYZQUAT
from pinocchio.rpy import rpyToMatrix

from hpp.corbaserver import loadServerPlugin
from hpp.corbaserver.manipulation import (
    Client,
    ConstraintGraph,
    ConstraintGraphFactory,
    Constraints,
    ProblemSolver,
    Rule,
)
from hpp.corbaserver.manipulation.robot import Robot as ParentRobot
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory


# ============================================================================
# Initialize CORBA Server
# ============================================================================

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()


# ============================================================================
# Utility Functions
# ============================================================================


def xyzrpy_to_xyzquat(xyzrpy):
    """
    Convert [x, y, z, roll, pitch, yaw] to [x, y, z, qx, qy, qz, qw].
    
    Args:
        xyzrpy: List of 6 values [x, y, z, roll, pitch, yaw]
        
    Returns:
        numpy.array: 7 values [x, y, z, qx, qy, qz, qw]
    """
    xyz = np.array(xyzrpy[:3])
    rpy = np.array(xyzrpy[3:])
    rotation_matrix = rpyToMatrix(rpy)
    se3 = SE3(rotation_matrix, xyz)
    return SE3ToXYZQUAT(se3)


# ============================================================================
# Robot and Object Definitions
# ============================================================================


class Robot(ParentRobot):
    """Spacelab composite robot (UR10 + VISPA)."""
    
    packageName = "spacelab_mock_hardware/description"
    urdfName = "allRobots_spacelab_robot"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__(self, compositeName, robotName, load=True, rootJointType="anchor"):
        super().__init__(compositeName, robotName, rootJointType, load)


class ModelConfig:
    """Base configuration for models."""
    
    packageName = "spacelab_mock_hardware/description"
    meshPackageName = "spacelab_mock_hardware/description"
    urdfSuffix = ""
    srdfSuffix = ""


class LabScene(ModelConfig):
    """Lab scene environment."""
    
    rootJointType = "anchor"
    urdfName = "gd_scene"


class GroundDemo(ModelConfig):
    """Ground demo environment."""
    
    rootJointType = "anchor"
    urdfName = "ground_demo"


class ScrewDriver(ModelConfig):
    """Screw driver tool object."""
    
    rootJointType = "freeflyer"
    urdfName = "screw_driver"


class FrameGripper(ModelConfig):
    """Frame gripper object."""
    
    rootJointType = "freeflyer"
    urdfName = "frame_gripper"


class CleatGripper(ModelConfig):
    """Cleat gripper object."""
    
    rootJointType = "freeflyer"
    urdfName = "cleat_gripper"


class RS(ModelConfig):
    """RS object."""
    
    rootJointType = "freeflyer"
    urdfName = "RS"


# ============================================================================
# Configuration Data
# ============================================================================


class InitialConfigurations:
    """Initial joint configurations for robots and objects."""
    
    # Robot joint configurations
    UR10 = [0, 0, 0, 0, 0, 0]
    VISPA = [0, 0, 0, 0, 0, 0]
    VISPA2 = [0, 0]
    
    # Object poses in XYZRPY format
    RS1 = [
        0.46567999999999976,
        2.0219499999999999,
        -0.34200800000000015,
        1.5707938223931903,
        -3.1415918612707121,
        2.0943948257717535,
    ]
    
    SCREW_DRIVER = [
        0.046500000888612281,
        1.3322769978599074,
        -1.2103000009948957,
        3.1415926529240905,
        -3.2988203261954171e-07,
        1.5707963267957175,
    ]
    
    FRAME_GRIPPER = [
        -0.13349999871947557,
        1.3322770029736009,
        -1.2132999986768855,
        -5.098912860707481e-09,
        -3.1415923241839976,
        1.5707963267958001,
    ]
    
    CLEAT_GRIPPER = [
        0.04650005078371261,
        1.510277021366528,
        -1.212900037140666,
        4.1022589876578654e-07,
        3.1415906427953018,
        0.78539700669219625,
    ]


class JointBounds:
    """Joint bounds for free-flying objects."""
    
    # Translation bounds [x_min, x_max, y_min, y_max, z_min, z_max]
    TRANSLATION = [-2, 2, -3, 3, -2, 2]
    
    # Quaternion bounds (slightly larger than unit quaternion)
    QUATERNION = [
        -1.0001, 1.0001,  # qx
        -1.0001, 1.0001,  # qy
        -1.0001, 1.0001,  # qz
        -1.0001, 1.0001,  # qw
    ]
    
    @classmethod
    def freeflyer(cls):
        """Get combined translation + quaternion bounds."""
        return cls.TRANSLATION + cls.QUATERNION


# ============================================================================
# Manipulation Configuration
# ============================================================================


class ManipulationConfig:
    """Configuration for manipulation planning with multiple grippers and objects."""
    
    # Define all grippers available on the robots
    GRIPPERS = {
        "ur10_gripper": "spacelab/ur10/gripper",
        "vispa_gripper": "spacelab/vispa/gripper",
    }
    
    # Define all objects and their handles
    OBJECTS = {
        "frame_gripper": {
            "handles": ["frame_gripper/h_FG_tool", "frame_gripper/h_FG_side"],
            "contact_surfaces": [],
        },
        "screw_driver": {
            "handles": ["screw_driver/h_SD_tool"],
            "contact_surfaces": [],
        },
        "cleat_gripper": {
            "handles": ["cleat_gripper/h_CG_tool"],
            "contact_surfaces": [],
        },
        "RS1": {
            "handles": ["RS1/h_RS_top", "RS1/h_RS_front"],
            "contact_surfaces": [],
        },
    }
    
    # Define valid gripper-handle pairs (None means allow all)
    # Format: {gripper_name: [list of allowed handles] or None for all}
    VALID_PAIRS = {
        "ur10_gripper": [
            "frame_gripper/h_FG_tool",
            "screw_driver/h_SD_tool",
            "RS1/h_RS_top",
        ],
        "vispa_gripper": [
            "cleat_gripper/h_CG_tool",
            "RS1/h_RS_front",
            "frame_gripper/h_FG_side",
        ],
    }
    
    # Environment contact surfaces
    ENV_CONTACTS = [
        "ground_demo/surface",
    ]


class RuleGenerator:
    """
    Automated rule generation for manipulation constraint graphs.
    
    This class helps create rules that:
    1. Allow only valid gripper-handle pairs
    2. Prevent redundant or impossible grasps
    3. Support complex multi-robot scenarios
    """
    
    @staticmethod
    def generate_grasp_rules(config):
        """
        Generate rules for valid grasps based on gripper-handle pairs.
        
        Args:
            config: ManipulationConfig class or instance
            
        Returns:
            list: List of Rule objects for the constraint graph
        """
        rules = []
        
        # Get configuration
        grippers = list(config.GRIPPERS.values())
        valid_pairs = config.VALID_PAIRS
        gripper_names = list(config.GRIPPERS.keys())
        
        # Strategy 1: Allow specific pairs, disallow everything else
        for gripper_key, gripper_path in config.GRIPPERS.items():
            allowed_handles = valid_pairs.get(gripper_key, None)
            
            if allowed_handles is None:
                # Allow all handles for this gripper
                rules.append(Rule([gripper_path], [".*"], True))
            elif allowed_handles:
                # Allow only specific handles
                for handle in allowed_handles:
                    rules.append(Rule([gripper_path], [handle], True))
            else:
                # Disallow all handles for this gripper
                rules.append(Rule([gripper_path], [".*"], False))
        
        # Strategy 2: Disallow grasps not explicitly allowed
        # This creates a whitelist approach
        all_handles = []
        for obj_data in config.OBJECTS.values():
            all_handles.extend(obj_data["handles"])
        
        for gripper_key, gripper_path in config.GRIPPERS.items():
            allowed_handles = valid_pairs.get(gripper_key, all_handles)
            if allowed_handles != all_handles:
                # Find handles NOT allowed for this gripper
                disallowed = set(all_handles) - set(allowed_handles)
                for handle in disallowed:
                    rules.append(Rule([gripper_path], [handle], False))
        
        return rules
    
    @staticmethod
    def generate_sequential_rules(config, task_sequence):
        """
        Generate rules for sequential manipulation tasks.
        
        Args:
            config: ManipulationConfig
            task_sequence: List of (gripper, handle) pairs defining the sequence
            
        Returns:
            list: Rules enforcing the task sequence
            
        Example:
            task_sequence = [
                ("ur10_gripper", "frame_gripper/h_FG_tool"),
                ("vispa_gripper", "cleat_gripper/h_CG_tool"),
            ]
        """
        rules = []
        
        # Allow only the pairs in the sequence
        for gripper_key, handle in task_sequence:
            gripper_path = config.GRIPPERS[gripper_key]
            rules.append(Rule([gripper_path], [handle], True))
        
        # Disallow everything else
        rules.append(Rule([".*"], [".*"], False))
        
        return rules
    
    @staticmethod
    def generate_priority_rules(config, priority_map):
        """
        Generate rules based on priority (higher priority = preferred).
        
        Args:
            config: ManipulationConfig
            priority_map: Dict mapping (gripper, handle) -> priority value
            
        Returns:
            list: Rules sorted by priority
            
        Example:
            priority_map = {
                ("ur10_gripper", "frame_gripper/h_FG_tool"): 10,
                ("ur10_gripper", "screw_driver/h_SD_tool"): 5,
            }
        """
        # Sort by priority (highest first)
        sorted_pairs = sorted(
            priority_map.items(),
            key=lambda x: x[1],
            reverse=True
        )
        
        rules = []
        for (gripper_key, handle), priority in sorted_pairs:
            gripper_path = config.GRIPPERS[gripper_key]
            rules.append(Rule([gripper_path], [handle], True))
        
        return rules
    
    @staticmethod
    def generate_collision_aware_rules(config, collision_groups):
        """
        Generate rules that prevent grasps leading to collisions.
        
        Args:
            config: ManipulationConfig
            collision_groups: Dict mapping object groups that collide when grasped together
            
        Returns:
            list: Rules preventing problematic simultaneous grasps
            
        Example:
            collision_groups = {
                "large_objects": ["RS1", "frame_gripper"],
                "small_tools": ["screw_driver", "cleat_gripper"],
            }
        """
        rules = []
        
        # First, allow all valid individual grasps
        for gripper_key, handles in config.VALID_PAIRS.items():
            gripper_path = config.GRIPPERS[gripper_key]
            for handle in handles:
                rules.append(Rule([gripper_path], [handle], True))
        
        # Then, add rules to prevent problematic combinations
        # (This would require more complex Rule syntax or custom constraints)
        
        return rules
    
    @staticmethod
    def print_rule_summary(rules, config):
        """
        Print a human-readable summary of the rules.
        
        Args:
            rules: List of Rule objects
            config: ManipulationConfig
        """
        print("\n" + "=" * 70)
        print("Constraint Graph Rules Summary")
        print("=" * 70)
        
        print(f"\nTotal rules: {len(rules)}")
        print("\nAllowed grasp pairs:")
        
        for rule in rules:
            if rule.link:  # If rule allows the grasp
                grippers = rule.grippers_
                handles = rule.handles_
                print(f"  ✓ {grippers} → {handles}")
        
        print("\nDisallowed grasp pairs:")
        for rule in rules:
            if not rule.link:  # If rule disallows the grasp
                grippers = rule.grippers_
                handles = rule.handles_
                print(f"  ✗ {grippers} → {handles}")
        
        print("=" * 70 + "\n")


# ============================================================================
# Setup Functions
# ============================================================================


def setup_robot():
    """Initialize robot and problem solver."""
    robot = Robot("spacelab-robots", "spacelab")
    ps = ProblemSolver(robot)
    ps.setErrorThreshold(1e-4)
    ps.setMaxIterProjection(40)
    return robot, ps


def load_environment(ps):
    """Load environment and objects into the viewer."""
    vf = ViewerFactory(ps)
    
    # Load environment
    vf.loadEnvironmentModel(GroundDemo, "ground_demo")
    
    # Load objects
    vf.loadObjectModel(RS, "RS1")
    vf.loadObjectModel(ScrewDriver, "screw_driver")
    vf.loadObjectModel(FrameGripper, "frame_gripper")
    vf.loadObjectModel(CleatGripper, "cleat_gripper")
    
    return vf


def setup_joint_bounds(robot):
    """Configure joint bounds for free-flying objects."""
    freeflyer_joints = [
        "RS1/root_joint",
        "screw_driver/root_joint",
        "frame_gripper/root_joint",
        "cleat_gripper/root_joint",
    ]
    
    bounds = JointBounds.freeflyer()
    for joint in freeflyer_joints:
        robot.setJointBounds(joint, bounds)


def build_initial_configuration():
    """Construct the initial configuration for all robots and objects."""
    config = InitialConfigurations
    
    # Start with robot configurations
    q_init = config.UR10 + config.VISPA2 + config.VISPA
    
    # Add object poses (convert XYZRPY to XYZQUAT)
    object_poses = [
        config.RS1,
        config.SCREW_DRIVER,
        config.FRAME_GRIPPER,
        config.CLEAT_GRIPPER,
    ]
    
    for pose_xyzrpy in object_poses:
        pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
        q_init += pose_xyzquat.tolist()
    
    return q_init


def create_constraint_graph(robot, ps, rule_strategy="auto"):
    """
    Create and initialize the constraint graph for manipulation.
    
    Args:
        robot: Robot instance
        ps: ProblemSolver instance
        rule_strategy: Rule generation strategy
            - "auto": Automatically generate rules from valid pairs
            - "all": Allow all gripper-handle combinations
            - "sequential": Use predefined task sequence
            - "custom": Provide custom rules (modify this function)
            
    Returns:
        tuple: (graph, factory, rules)
    """
    
    config = ManipulationConfig
    
    # Extract data from configuration
    grippers = list(config.GRIPPERS.values())
    objects = list(config.OBJECTS.keys())
    
    handles_per_object = [
        config.OBJECTS[obj]["handles"] for obj in objects
    ]
    
    contact_surfaces_per_object = [
        config.OBJECTS[obj]["contact_surfaces"] for obj in objects
    ]
    
    env_contact_surfaces = config.ENV_CONTACTS
    
    # Generate rules based on strategy
    rule_gen = RuleGenerator()
    
    if rule_strategy == "auto":
        # Automatically generate from valid pairs configuration
        rules = rule_gen.generate_grasp_rules(config)
        
    elif rule_strategy == "all":
        # Allow all combinations
        rules = [Rule([".*"], [".*"], True)]
        
    elif rule_strategy == "sequential":
        # Define your task sequence here
        task_sequence = [
            ("ur10_gripper", "frame_gripper/h_FG_tool"),
            ("vispa_gripper", "cleat_gripper/h_CG_tool"),
        ]
        rules = rule_gen.generate_sequential_rules(config, task_sequence)
        
    elif rule_strategy == "priority":
        # Define priority map
        priority_map = {
            ("ur10_gripper", "frame_gripper/h_FG_tool"): 10,
            ("ur10_gripper", "screw_driver/h_SD_tool"): 8,
            ("vispa_gripper", "cleat_gripper/h_CG_tool"): 9,
        }
        rules = rule_gen.generate_priority_rules(config, priority_map)
        
    else:  # custom
        # Define custom rules manually
        rules = [
            Rule([".*"], [".*"], True),
        ]
    
    # Print rule summary for debugging
    if rule_strategy != "all":
        rule_gen.print_rule_summary(rules, config)
    
    # Create constraint graph
    graph = ConstraintGraph(robot, "graph")
    factory = ConstraintGraphFactory(graph)
    
    # Configure factory
    factory.setGrippers(grippers)
    factory.environmentContacts(env_contact_surfaces)
    factory.setObjects(objects, handles_per_object, contact_surfaces_per_object)
    factory.setRules(rules)
    
    # Generate and initialize
    print(f"\nGenerating constraint graph with {len(rules)} rules...")
    factory.generate()
    
    # Add any additional constraints if needed
    constraints = []
    if constraints:
        graph.addConstraints(
            graph=True,
            constraints=Constraints(numConstraints=constraints)
        )
    
    graph.initialize()
    print("Constraint graph initialized successfully!")
    
    return graph, factory, rules


# ============================================================================
# Main Execution
# ============================================================================


def main(rule_strategy="auto"):
    """
    Main execution function.
    
    Args:
        rule_strategy: Rule generation strategy for constraint graph
            - "auto": Use ManipulationConfig.VALID_PAIRS
            - "all": Allow all combinations
            - "sequential": Use predefined sequence
            - "priority": Use priority-based rules
    """
    
    print("=" * 70)
    print("Spacelab Manipulation Planning")
    print("=" * 70)
    print(f"Rule Strategy: {rule_strategy}")
    
    # Setup robot and problem solver
    print("\n1. Setting up robot and problem solver...")
    robot, ps = setup_robot()
    
    # Load environment
    print("2. Loading environment and objects...")
    vf = load_environment(ps)
    
    # Configure joint bounds
    print("3. Configuring joint bounds...")
    setup_joint_bounds(robot)
    
    # Build initial configuration
    print("4. Building initial configuration...")
    q_init = build_initial_configuration()
    
    # Create constraint graph
    print("5. Creating constraint graph...")
    graph, factory, rules = create_constraint_graph(robot, ps, rule_strategy)
    
    # TODO: Define goal configuration
    # q_goal = build_goal_configuration()
    
    # Set up and solve planning problem
    print("6. Setting up planning problem...")
    ps.setInitialConfig(q_init)
    # ps.addGoalConfig(q_goal)
    
    # Uncomment to solve:
    # print("7. Solving...")
    # ps.solve()
    # print("Solution found!")
    
    # Visualization
    print("\n8. Starting visualization...")
    v = vf.createViewer()
    v(q_init)
    pp = PathPlayer(v)
    
    print("\n" + "=" * 70)
    print("Setup complete!")
    print("=" * 70)
    print("\nNext steps:")
    print("- Define q_goal configuration")
    print("- Call ps.solve() to find a path")
    print("- Use pp(0) to visualize the solution")
    print("\nTo change rule strategy, call:")
    print("  main(rule_strategy='auto|all|sequential|priority')")
    
    return robot, ps, vf, graph, factory, rules, v, pp


# ============================================================================
# Execute
# ============================================================================

if __name__ == "__main__":
    # Choose your rule strategy:
    # "auto" - use valid pairs from ManipulationConfig
    # "all" - allow all combinations
    # "sequential" - use predefined task sequence
    # "priority" - use priority-based rules
    robot, ps, vf, graph, factory, rules, viewer, path_player = main(rule_strategy="auto")
