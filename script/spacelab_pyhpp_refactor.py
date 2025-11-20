"""
Spacelab manipulation planning script using PyHPP (Python bindings).

This script sets up a multi-robot manipulation scenario with:
- UR10 and VISPA robots
- Multiple objects: RS1, ScrewDriver, FrameGripper, CleatGripper
- Motion planning using PyHPP instead of CORBA server

Key differences from corbaserver version:
- Uses pyhpp.pinocchio.Device instead of hpp.corbaserver.Robot
- Direct Python API instead of CORBA calls
- More Pythonic interface with better performance
"""

import numpy as np
import pinocchio as pin
from pinocchio import SE3, Quaternion
from pinocchio.rpy import rpyToMatrix

from pyhpp.manipulation import Device, urdf, Graph, ManipulationPlanner
from pyhpp.manipulation import createProgressiveProjector, Problem
from pyhpp.core import (
    DiffusingPlanner,
    BiRRTPlanner,
    VisibilityPrmPlanner,
    BiRrtStar,
    kPrmStar,
    createDichotomy,
)
from pyhpp.gepetto.viewer import Viewer


# ============================================================================
# Utility Functions
# ============================================================================


def xyzrpy_to_se3(xyzrpy):
    """
    Convert [x, y, z, roll, pitch, yaw] to SE3 transformation.
    
    Args:
        xyzrpy: List of 6 values [x, y, z, roll, pitch, yaw]
        
    Returns:
        pinocchio.SE3: SE3 transformation
    """
    xyz = np.array(xyzrpy[:3])
    rpy = np.array(xyzrpy[3:])
    rotation_matrix = rpyToMatrix(rpy)
    return SE3(rotation_matrix, xyz)


def se3_to_xyzquat(se3):
    """
    Convert SE3 to [x, y, z, qx, qy, qz, qw] configuration.
    
    Args:
        se3: pinocchio.SE3 transformation
        
    Returns:
        numpy.array: 7 values [x, y, z, qx, qy, qz, qw]
    """
    xyz = se3.translation
    quat = Quaternion(se3.rotation)
    return np.concatenate([xyz, quat.coeffs()])


def xyzrpy_to_xyzquat(xyzrpy):
    """
    Convert [x, y, z, roll, pitch, yaw] to [x, y, z, qx, qy, qz, qw].
    
    Args:
        xyzrpy: List of 6 values [x, y, z, roll, pitch, yaw]
        
    Returns:
        numpy.array: 7 values [x, y, z, qx, qy, qz, qw]
    """
    se3 = xyzrpy_to_se3(xyzrpy)
    return se3_to_xyzquat(se3)


# ============================================================================
# Model Configuration
# ============================================================================


class URDFPaths:
    """URDF and SRDF file paths for all models."""
    
    # Base package
    PACKAGE = "package://spacelab_mock_hardware/description"
    
    # Robot models
    ROBOT_URDF = f"{PACKAGE}/urdf/allRobots_spacelab_robot.urdf"
    ROBOT_SRDF = f"{PACKAGE}/srdf/allRobots_spacelab_robot.srdf"
    
    # Environment
    GROUND_DEMO_URDF = f"{PACKAGE}/urdf/ground_demo.urdf"
    LAB_SCENE_URDF = f"{PACKAGE}/urdf/gd_scene.urdf"
    
    # Objects
    RS_URDF = f"{PACKAGE}/urdf/RS.urdf"
    SCREW_DRIVER_URDF = f"{PACKAGE}/urdf/screw_driver.urdf"
    FRAME_GRIPPER_URDF = f"{PACKAGE}/urdf/frame_gripper.urdf"
    CLEAT_GRIPPER_URDF = f"{PACKAGE}/urdf/cleat_gripper.urdf"


class RobotJoints:
    """Joint names for the composite robot."""
    
    # UR10 joints (6 DOF)
    UR10 = [
        "ur10/shoulder_pan_joint",
        "ur10/shoulder_lift_joint",
        "ur10/elbow_joint",
        "ur10/wrist_1_joint",
        "ur10/wrist_2_joint",
        "ur10/wrist_3_joint",
    ]
    
    # VISPA joints (8 DOF: 2 base + 6 arm)
    VISPA_BASE = [
        "vispa/base_x",
        "vispa/base_y",
    ]
    
    VISPA_ARM = [
        "vispa/joint1",
        "vispa/joint2",
        "vispa/joint3",
        "vispa/joint4",
        "vispa/joint5",
        "vispa/joint6",
    ]
    
    # Object root joints (freeflyer: 7 DOF each)
    RS1_ROOT = "RS1/root_joint"
    SCREW_DRIVER_ROOT = "screw_driver/root_joint"
    FRAME_GRIPPER_ROOT = "frame_gripper/root_joint"
    CLEAT_GRIPPER_ROOT = "cleat_gripper/root_joint"
    
    @classmethod
    def all_robot_joints(cls):
        """Get all robot joint names."""
        return cls.UR10 + cls.VISPA_BASE + cls.VISPA_ARM
    
    @classmethod
    def all_object_roots(cls):
        """Get all object root joint names."""
        return [
            cls.RS1_ROOT,
            cls.SCREW_DRIVER_ROOT,
            cls.FRAME_GRIPPER_ROOT,
            cls.CLEAT_GRIPPER_ROOT,
        ]


# ============================================================================
# Configuration Data
# ============================================================================


class InitialConfigurations:
    """Initial joint configurations for robots and objects."""
    
    # Robot joint configurations (in radians)
    UR10 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 DOF
    VISPA_BASE = [0.0, 0.0]  # 2 DOF base
    VISPA_ARM = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 DOF arm
    
    # Object poses in XYZRPY format [x, y, z, roll, pitch, yaw]
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
    """Joint bounds for robots and free-flying objects."""
    
    # Robot joint limits (example values - adjust based on actual robot)
    UR10_LIMITS = [
        (-2 * np.pi, 2 * np.pi),  # All UR10 joints
    ] * 6
    
    VISPA_BASE_LIMITS = [
        (-5.0, 5.0),  # x translation
        (-5.0, 5.0),  # y translation
    ]
    
    VISPA_ARM_LIMITS = [
        (-np.pi, np.pi),  # All VISPA joints
    ] * 6
    
    # Freeflyer bounds for objects
    # Translation bounds [x_min, x_max, y_min, y_max, z_min, z_max]
    TRANSLATION_BOUNDS = [
        (-2.0, 2.0),  # x
        (-3.0, 3.0),  # y
        (-2.0, 2.0),  # z
    ]
    
    # Quaternion bounds (must contain unit quaternion)
    QUATERNION_BOUNDS = [
        (-1.0001, 1.0001),  # qx
        (-1.0001, 1.0001),  # qy
        (-1.0001, 1.0001),  # qz
        (-1.0001, 1.0001),  # qw
    ]
    
    @classmethod
    def freeflyer_bounds(cls):
        """Get combined translation + quaternion bounds for freeflyer."""
        return cls.TRANSLATION_BOUNDS + cls.QUATERNION_BOUNDS
    
    @classmethod
    def all_robot_bounds(cls):
        """Get bounds for all robot joints."""
        return cls.UR10_LIMITS + cls.VISPA_BASE_LIMITS + cls.VISPA_ARM_LIMITS


class PlannerConfig:
    """Configuration for motion planning algorithms."""
    
    # Planner types
    DIFFUSING = "diffusing"
    BIRRT = "birrt"
    VISIBILITY_PRM = "visibility_prm"
    BIRRT_STAR = "birrt_star"
    KPRM_STAR = "kprm_star"
    
    # Default parameters
    MAX_ITERATIONS = 10000
    STEP_SIZE = 0.1
    COLLISION_CHECK_STEP = 0.01
    
    # Planning configurations
    PLANNERS = {
        DIFFUSING: {
            "class": DiffusingPlanner,
            "params": {},
        },
        BIRRT: {
            "class": BiRRTPlanner,
            "params": {},
        },
        VISIBILITY_PRM: {
            "class": VisibilityPrmPlanner,
            "params": {},
        },
        BIRRT_STAR: {
            "class": BiRrtStar,
            "params": {},
        },
        KPRM_STAR: {
            "class": kPrmStar,
            "params": {},
        },
    }


# ============================================================================
# Robot Setup
# ============================================================================


class SpacelabRobot:
    """Wrapper class for Spacelab robot with PyHPP."""
    
    def __init__(self, load_objects=True):
        """
        Initialize Spacelab robot.
        
        Args:
            load_objects: Whether to load manipulatable objects
        """
        self.device = Device("spacelab")
        self.load_objects_flag = load_objects
        
        # Load robot
        self._load_robot()
        
        # Load environment
        self._load_environment()
        
        # Load objects if requested
        if load_objects:
            self._load_objects()
        
        # Store model information
        self.model = self.device.model()
        self.data = self.device.data()
        self.nq = self.model.nq
        self.nv = self.model.nv
        
    def _load_robot(self):
        """Load the composite robot (UR10 + VISPA)."""
        urdf.loadModel(
            self.device,
            0,
            "spacelab-robots",
            "anchor",
            URDFPaths.ROBOT_URDF,
            URDFPaths.ROBOT_SRDF,
            SE3.Identity()
        )
        print("✓ Loaded Spacelab robot")
        
    def _load_environment(self):
        """Load the environment scene."""
        urdf.loadModel(
            self.device,
            0,
            "spacelab-scene",
            "anchor",
            URDFPaths.GROUND_DEMO_URDF,
            "",
            SE3.Identity()
        )
        print("✓ Loaded environment")
        
    def _load_objects(self):
        """Load manipulatable objects."""
        objects = [
            ("RS1", URDFPaths.RS_URDF),
            ("screw_driver", URDFPaths.SCREW_DRIVER_URDF),
            ("frame_gripper", URDFPaths.FRAME_GRIPPER_URDF),
            ("cleat_gripper", URDFPaths.CLEAT_GRIPPER_URDF),
        ]
        
        for name, urdf_path in objects:
            # Objects are loaded as freeflyer
            urdf.loadModel(
                self.device,
                0,
                name,
                "freeflyer",
                urdf_path,
                "",
                SE3.Identity()
            )
        
        print(f"✓ Loaded {len(objects)} objects")
        
    def get_neutral_config(self):
        """Get neutral/home configuration."""
        return pin.neutral(self.model)
    
    def get_random_config(self):
        """Get random configuration within bounds."""
        return pin.randomConfiguration(self.model)
    
    def set_joint_bounds(self):
        """Set joint bounds for the robot."""
        # This would require accessing the model's bounds
        # PyHPP handles this differently than corbaserver
        pass


# ============================================================================
# Configuration Building
# ============================================================================


def build_initial_configuration(robot):
    """
    Construct the initial configuration for all robots and objects.
    
    Args:
        robot: SpacelabRobot instance
        
    Returns:
        numpy.array: Initial configuration vector
    """
    config = InitialConfigurations
    
    # Start with robot configurations
    q_robot = np.array(config.UR10 + config.VISPA_BASE + config.VISPA_ARM)
    
    # Add object poses (convert XYZRPY to XYZQUAT for freeflyer)
    object_poses = [
        config.RS1,
        config.SCREW_DRIVER,
        config.FRAME_GRIPPER,
        config.CLEAT_GRIPPER,
    ]
    
    q_objects = []
    for pose_xyzrpy in object_poses:
        pose_xyzquat = xyzrpy_to_xyzquat(pose_xyzrpy)
        q_objects.extend(pose_xyzquat)
    
    # Combine robot and object configurations
    q_init = np.concatenate([q_robot, q_objects])
    
    return q_init


def build_goal_configuration(robot, object_offsets=None):
    """
    Build a goal configuration with modified object poses.
    
    Args:
        robot: SpacelabRobot instance
        object_offsets: Dict of object_name -> [dx, dy, dz] offsets
        
    Returns:
        numpy.array: Goal configuration vector
    """
    if object_offsets is None:
        object_offsets = {
            "RS1": [0.2, 0.0, 0.0],  # Move RS1 20cm in x
        }
    
    # Start with initial configuration
    q_goal = build_initial_configuration(robot)
    
    # Modify object poses based on offsets
    # This requires knowing the joint indices for each object
    # For now, return a simple modification
    
    return q_goal


# ============================================================================
# Motion Planning
# ============================================================================


class MotionPlanner:
    """Wrapper for motion planning with PyHPP."""
    
    def __init__(self, robot, planner_type="birrt"):
        """
        Initialize motion planner.
        
        Args:
            robot: SpacelabRobot instance
            planner_type: Type of planner to use
        """
        self.robot = robot
        self.planner_type = planner_type
        
        # Create problem (CRITICAL: Pass robot.device, not robot wrapper)
        self.problem = Problem(robot.device)
        
        # Create planner
        planner_info = PlannerConfig.PLANNERS.get(planner_type)
        if planner_info:
            self.planner = planner_info["class"]()
        else:
            raise ValueError(f"Unknown planner type: {planner_type}")
        
        print(f"✓ Created planner: {planner_type}")
        
    def set_initial_config(self, q_init):
        """Set initial configuration."""
        self.problem.initConfig(q_init)
        self.q_init = q_init
        
    def set_goal_config(self, q_goal):
        """Set goal configuration."""
        self.problem.addGoalConfig(q_goal)
        self.q_goal = q_goal
        
    def solve(self, max_iterations=None):
        """
        Solve the motion planning problem.
        
        Args:
            max_iterations: Maximum planning iterations
            
        Returns:
            bool: True if solution found
        """
        if max_iterations is None:
            max_iterations = PlannerConfig.MAX_ITERATIONS
        
        print(f"Planning with {self.planner_type}...")
        print(f"Max iterations: {max_iterations}")
        
        # Run planner
        success = self.planner.solve(
            self.problem,
            max_iterations
        )
        
        if success:
            print("✓ Solution found!")
            self.solution_path = self.planner.path()
            return True
        else:
            print("✗ No solution found")
            return False
    
    def get_path(self):
        """Get the solution path."""
        if hasattr(self, 'solution_path'):
            return self.solution_path
        return None
    
    def path_length(self):
        """Get path length."""
        if hasattr(self, 'solution_path'):
            return self.solution_path.length()
        return 0


class ManipulationPlannerWrapper:
    """Wrapper for manipulation planning with constraint graphs."""
    
    def __init__(self, robot):
        """
        Initialize manipulation planner.
        
        Args:
            robot: SpacelabRobot instance
        """
        self.robot = robot
        self.problem = Problem(robot.device)
        
        # # Set up path validation and projection for constrained planning
        self.problem.pathValidation = createDichotomy(robot.device.asPinDevice(), 0)
        self.problem.pathProjector = createProgressiveProjector(
            self.problem.distance(),
            self.problem.steeringMethod(),
            0.01
        )
        
        # Create constraint graph
        self.graph = Graph("manipulation_graph", robot.device, self.problem)
        
        print("✓ Created manipulation planner")
        
    def create_state(self, name, is_waypoint=False, priority=0):
        """
        Create a state (node) in the constraint graph.
        
        Args:
            name: State name
            is_waypoint: Whether this is a waypoint state
            priority: State priority
            
        Returns:
            State index
        """
        state = self.graph.createState(name, is_waypoint, priority)
        print(f"  Created state: {name}")
        return state
        
    def create_edge(self, from_state, to_state, name, weight=1, containing_state=None):
        """
        Create an edge (transition) between states.
        
        Args:
            from_state: Source state object
            to_state: Target state object
            name: Edge name
            weight: Edge weight
            containing_state: State whose constraints apply to path (default: from_state)
            
        Returns:
            Edge object
        """
        if containing_state is None:
            containing_state = from_state
            
        edge = self.graph.createTransition(
            from_state, to_state, name, weight, containing_state
        )
        print(f"  Created edge: {name}")
        return edge
        
    def initialize_graph(self):
        """Initialize the constraint graph."""
        # Configure graph parameters BEFORE initialize
        self.graph.maxIterations(100)
        self.graph.errorThreshold(1e-5)
        self.graph.initialize()
        print("✓ Graph initialized")
        
    def apply_state_constraints(self, state, q):
        """
        Project configuration onto state constraints.
        
        Args:
            state: State object
            q: Configuration to project
            
        Returns:
            ConstraintResult: Result object with success, configuration, error
        """
        result = self.graph.applyStateConstraints(state, q)
        return result
        
    def set_initial_config(self, q_init, state=None):
        """
        Set initial configuration, optionally projecting onto state.
        
        Args:
            q_init: Initial configuration
            state: Optional state to project onto
        """
        if state is not None:
            result = self.apply_state_constraints(state, q_init)
            if result.success:
                q_init = result.configuration
            else:
                print(f"Warning: Failed to project initial config")
                print(f"  Error: {result.error}")
        
        self.problem.initConfig(q_init)
        self.q_init = q_init
        
    def set_goal_config(self, q_goal, state=None):
        """
        Set goal configuration, optionally projecting onto state.
        
        Args:
            q_goal: Goal configuration
            state: Optional state to project onto
        """
        if state is not None:
            result = self.apply_state_constraints(state, q_goal)
            if result.success:
                q_goal = result.configuration
            else:
                print(f"Warning: Failed to project goal config")
                print(f"  Error: {result.error}")
        
        self.problem.addGoalConfig(q_goal)
        self.q_goal = q_goal
        
    def solve(self, max_iterations=5000):
        """
        Solve the manipulation planning problem.
        
        Args:
            max_iterations: Maximum planning iterations
            
        Returns:
            bool: True if solution found
        """
        # Set constraint graph for the problem
        self.problem.constraintGraph(self.graph)
        
        # Create manipulation planner
        planner = ManipulationPlanner(self.problem)
        planner.maxIterations(max_iterations)
        
        print(f"Solving manipulation problem...")
        print(f"Max iterations: {max_iterations}")
        
        # Solve
        success = planner.solve()
        
        if success:
            print("✓ Solution found!")
            self.solution_path = planner.path()
            return True
        else:
            print("✗ No solution found")
            return False
            
    def get_path(self):
        """Get the solution path."""
        if hasattr(self, 'solution_path'):
            return self.solution_path
        return None


# ============================================================================
# Visualization
# ============================================================================


class SpacelabViewer:
    """Wrapper for visualization with Gepetto viewer."""
    
    def __init__(self, robot):
        """
        Initialize viewer.
        
        Args:
            robot: SpacelabRobot instance
        """
        self.robot = robot
        self.viewer = Viewer(robot.device)
        print("✓ Viewer initialized")
        
    def display(self, q):
        """
        Display configuration.
        
        Args:
            q: Configuration vector
        """
        self.viewer(q)
        
    def play_path(self, path, dt=0.01):
        """
        Play a path animation.
        
        Args:
            path: Path object or list of configurations
            dt: Time step between configurations
        """
        import time
        
        if hasattr(path, 'length'):
            # It's a path object
            t = 0
            while t <= path.length():
                q = path(t)
                self.display(q)
                time.sleep(dt)
                t += dt
        else:
            # It's a list of configurations
            for q in path:
                self.display(q)
                time.sleep(dt)


# ============================================================================
# Main Execution
# ============================================================================


def main(planner_type="birrt", use_manipulation=False, solve=False):
    """
    Main execution function.
    
    Args:
        planner_type: Type of planner to use (for motion planning)
        use_manipulation: Whether to use manipulation planning with constraint graph
        solve: Whether to solve the planning problem
    """
    print("=" * 70)
    print("Spacelab Manipulation Planning (PyHPP)")
    print("=" * 70)
    
    if use_manipulation:
        print("Mode: Manipulation Planning (with constraint graph)")
    else:
        print(f"Mode: Motion Planning (planner: {planner_type})")
    
    # 1. Setup robot
    print("\n1. Setting up robot...")
    robot = SpacelabRobot(load_objects=True)
    
    # 2. Build configurations
    print("\n2. Building configurations...")
    q_init = build_initial_configuration(robot)
    q_goal = build_goal_configuration(robot)
    
    print(f"   Initial config: {q_init.shape}")
    print(f"   Goal config: {q_goal.shape}")
    
    # 3. Create planner
    solution_found = False
    
    if use_manipulation:
        print("\n3. Creating manipulation planner with constraint graph...")
        planner = ManipulationPlannerWrapper(robot)
        
        # Create simple constraint graph
        # Example: placement state
        state_placement = planner.create_state("placement", False, 0)
        planner.initialize_graph()
        
        # Set configurations (project onto state constraints)
        planner.set_initial_config(q_init, state_placement)
        planner.set_goal_config(q_goal, state_placement)
        
        # Solve if requested
        if solve:
            print("\n4. Solving manipulation planning problem...")
            solution_found = planner.solve()
            
            if solution_found:
                path_length = planner.get_path().length()
                print(f"   Path length: {path_length:.3f}")
    else:
        print("\n3. Creating motion planner...")
        planner = MotionPlanner(robot, planner_type)
        planner.set_initial_config(q_init)
        planner.set_goal_config(q_goal)
        
        # Solve if requested
        if solve:
            print("\n4. Solving motion planning problem...")
            solution_found = planner.solve()
            
            if solution_found:
                path_length = planner.path_length()
                print(f"   Path length: {path_length:.3f}")
    
    # 5. Visualization
    step = 5 if not solve else 6
    print(f"\n{step}. Starting visualization...")
    viewer = SpacelabViewer(robot)
    viewer.display(q_init)
    
    print("\n" + "=" * 70)
    print("Setup complete!")
    print("=" * 70)
    
    if not solve:
        print("\nTo solve the planning problem, call:")
        if use_manipulation:
            print("  main(use_manipulation=True, solve=True)")
        else:
            print("  main(planner_type='birrt', solve=True)")
    
    print("\nAvailable commands:")
    print("  viewer.display(q_init)  - Display initial config")
    print("  viewer.display(q_goal)  - Display goal config")
    
    if solution_found:
        print("  viewer.play_path(planner.get_path())  - Animate solution")
        print("  planner.get_path()  - Get solution path")
    
    return robot, planner, viewer


# ============================================================================
# Execute
# ============================================================================

if __name__ == "__main__":
    # Motion planning mode
    # robot, planner, viewer = main(planner_type="birrt", use_manipulation=False, solve=False)
    
    # Manipulation planning mode (with constraint graph)
    robot, planner, viewer = main(use_manipulation=True, solve=False)
    
    # To solve and animate:
    # robot, planner, viewer = main(use_manipulation=True, solve=True)
    # viewer.play_path(planner.get_path())
