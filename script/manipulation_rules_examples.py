"""
Examples of how to use the automated rule generation system for manipulation planning.

This file demonstrates different strategies for creating constraint graph rules
when dealing with multiple grippers and handles.
"""

from spacelab_refactor import ManipulationConfig, RuleGenerator

# ============================================================================
# Example 1: Basic Configuration
# ============================================================================

print("=" * 70)
print("Example 1: Basic Auto-Generated Rules")
print("=" * 70)

# Configuration is defined in ManipulationConfig class
config = ManipulationConfig

# Generate rules automatically based on VALID_PAIRS
rule_gen = RuleGenerator()
rules = rule_gen.generate_grasp_rules(config)

print(f"\nGenerated {len(rules)} rules:")
for i, rule in enumerate(rules, 1):
    print(f"{i}. Gripper: {rule.grippers_}, Handle: {rule.handles_}, Allow: {rule.link}")


# ============================================================================
# Example 2: Custom Configuration for Different Scenario
# ============================================================================

print("\n" + "=" * 70)
print("Example 2: Custom Configuration")
print("=" * 70)


class CustomManipulationConfig:
    """Custom configuration for a different manipulation scenario."""
    
    GRIPPERS = {
        "left_arm": "robot/left_gripper",
        "right_arm": "robot/right_gripper",
    }
    
    OBJECTS = {
        "box": {
            "handles": ["box/h_left", "box/h_right", "box/h_top"],
            "contact_surfaces": ["box/bottom"],
        },
        "tool": {
            "handles": ["tool/h_handle"],
            "contact_surfaces": [],
        },
        "part": {
            "handles": ["part/h_mount"],
            "contact_surfaces": ["part/base"],
        },
    }
    
    # Left arm: Can grasp left side and top
    # Right arm: Can grasp right side and tools
    VALID_PAIRS = {
        "left_arm": [
            "box/h_left",
            "box/h_top",
            "part/h_mount",
        ],
        "right_arm": [
            "box/h_right",
            "tool/h_handle",
        ],
    }
    
    ENV_CONTACTS = ["table/surface"]


# Generate rules for custom config
custom_rules = rule_gen.generate_grasp_rules(CustomManipulationConfig)
print(f"\nGenerated {len(custom_rules)} rules for custom scenario")


# ============================================================================
# Example 3: Sequential Task Planning
# ============================================================================

print("\n" + "=" * 70)
print("Example 3: Sequential Task Rules")
print("=" * 70)

# Define a specific task sequence
task_sequence = [
    ("ur10_gripper", "screw_driver/h_SD_tool"),  # 1. UR10 picks up screwdriver
    ("vispa_gripper", "RS1/h_RS_front"),         # 2. VISPA grasps RS1
    ("ur10_gripper", "frame_gripper/h_FG_tool"), # 3. UR10 switches to frame gripper
]

sequential_rules = rule_gen.generate_sequential_rules(config, task_sequence)
print(f"\nGenerated {len(sequential_rules)} rules for sequential task:")
for i, (gripper, handle) in enumerate(task_sequence, 1):
    print(f"  Step {i}: {gripper} → {handle}")


# ============================================================================
# Example 4: Priority-Based Rules
# ============================================================================

print("\n" + "=" * 70)
print("Example 4: Priority-Based Rules")
print("=" * 70)

# Assign priorities to different grasps
priority_map = {
    ("ur10_gripper", "frame_gripper/h_FG_tool"): 10,  # Highest priority
    ("vispa_gripper", "cleat_gripper/h_CG_tool"): 9,
    ("ur10_gripper", "screw_driver/h_SD_tool"): 8,
    ("vispa_gripper", "RS1/h_RS_front"): 7,
    ("ur10_gripper", "RS1/h_RS_top"): 5,
}

priority_rules = rule_gen.generate_priority_rules(config, priority_map)
print(f"\nGenerated {len(priority_rules)} priority-based rules")
print("Rules will be tried in order of priority (highest first)")


# ============================================================================
# Example 5: Dynamic Rule Generation Based on Object Properties
# ============================================================================

print("\n" + "=" * 70)
print("Example 5: Dynamic Rule Generation")
print("=" * 70)


def generate_rules_by_weight(config):
    """
    Generate rules based on object weight.
    Heavy objects can only be grasped by specific grippers.
    """
    from hpp.corbaserver.manipulation import Rule
    
    # Define object weights (example)
    object_weights = {
        "frame_gripper": 2.0,  # kg
        "screw_driver": 0.5,
        "cleat_gripper": 1.0,
        "RS1": 5.0,  # Heavy object
    }
    
    # Define gripper capacities
    gripper_capacities = {
        "ur10_gripper": 10.0,  # Can lift up to 10kg
        "vispa_gripper": 3.0,  # Can lift up to 3kg
    }
    
    rules = []
    
    # Generate rules based on weight constraints
    for obj_name, weight in object_weights.items():
        handles = config.OBJECTS[obj_name]["handles"]
        
        for gripper_key, capacity in gripper_capacities.items():
            gripper_path = config.GRIPPERS[gripper_key]
            
            if weight <= capacity:
                # Gripper can handle this object
                for handle in handles:
                    # Also check if it's in valid pairs
                    if gripper_key in config.VALID_PAIRS:
                        if handle in config.VALID_PAIRS[gripper_key]:
                            rules.append(Rule([gripper_path], [handle], True))
                            print(f"  ✓ {gripper_key} can grasp {handle} ({weight}kg <= {capacity}kg)")
            else:
                # Gripper cannot handle this object
                for handle in handles:
                    rules.append(Rule([gripper_path], [handle], False))
                    print(f"  ✗ {gripper_key} cannot grasp {handle} ({weight}kg > {capacity}kg)")
    
    return rules


weight_based_rules = generate_rules_by_weight(config)
print(f"\nGenerated {len(weight_based_rules)} weight-based rules")


# ============================================================================
# Example 6: Collision-Aware Rules
# ============================================================================

print("\n" + "=" * 70)
print("Example 6: Collision-Aware Rules")
print("=" * 70)


def generate_collision_aware_rules(config):
    """
    Generate rules that consider potential collisions.
    Some objects cannot be grasped from certain handles due to workspace limits.
    """
    from hpp.corbaserver.manipulation import Rule
    
    # Define workspace limitations
    # Format: {gripper: {handle: reachable}}
    workspace_constraints = {
        "ur10_gripper": {
            "frame_gripper/h_FG_tool": True,
            "frame_gripper/h_FG_side": False,  # Unreachable
            "screw_driver/h_SD_tool": True,
            "RS1/h_RS_top": True,
            "RS1/h_RS_front": False,  # Would collide with environment
        },
        "vispa_gripper": {
            "cleat_gripper/h_CG_tool": True,
            "RS1/h_RS_front": True,
            "frame_gripper/h_FG_side": True,
            "RS1/h_RS_top": False,  # Unreachable from VISPA's workspace
        },
    }
    
    rules = []
    
    for gripper_key, reachability in workspace_constraints.items():
        gripper_path = config.GRIPPERS[gripper_key]
        
        for handle, is_reachable in reachability.items():
            rules.append(Rule([gripper_path], [handle], is_reachable))
            status = "✓ reachable" if is_reachable else "✗ unreachable/collision"
            print(f"  {status}: {gripper_key} → {handle}")
    
    return rules


collision_rules = generate_collision_aware_rules(config)
print(f"\nGenerated {len(collision_rules)} collision-aware rules")


# ============================================================================
# Summary and Best Practices
# ============================================================================

print("\n" + "=" * 70)
print("SUMMARY: Best Practices for Rule Generation")
print("=" * 70)

summary = """
1. **Start Simple**:
   - Begin with "all" strategy to test basic setup
   - Then refine to "auto" with VALID_PAIRS

2. **Use Configuration Classes**:
   - Define all grippers, objects, and handles in one place
   - Makes it easy to modify and maintain

3. **Leverage Automation**:
   - Use RuleGenerator for common patterns
   - Extend with custom generators for specific needs

4. **Consider Constraints**:
   - Weight/payload limits
   - Workspace reachability
   - Collision avoidance
   - Task sequence requirements

5. **Debugging**:
   - Use print_rule_summary() to verify rules
   - Start with restrictive rules, then relax as needed
   - Test with simple scenarios first

6. **Performance**:
   - Fewer rules = faster graph generation
   - But too few rules = infeasible paths
   - Balance between flexibility and efficiency

7. **Documentation**:
   - Comment why specific pairs are allowed/disallowed
   - Document task-specific constraints
   - Keep examples for future reference
"""

print(summary)

print("\n" + "=" * 70)
print("To use in your planning script:")
print("=" * 70)
print("""
# Option 1: Use auto-generated rules
robot, ps, vf, graph, factory, rules, v, pp = main(rule_strategy="auto")

# Option 2: Use sequential rules
robot, ps, vf, graph, factory, rules, v, pp = main(rule_strategy="sequential")

# Option 3: Use priority-based rules
robot, ps, vf, graph, factory, rules, v, pp = main(rule_strategy="priority")

# Option 4: Customize in create_constraint_graph() function
""")
