from hpp.corbaserver.manipulation.ur5 import Robot
from hpp.corbaserver.manipulation import ConstraintGraph as Parent
from hpp.corbaserver.manipulation import ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from hpp.gepetto import PathPlayer

Robot.urdfName = "ur5_gripper"
Robot.urdfSuffix = ""
Robot.srdfSuffix = ""

class Pokeball (object):
  rootJointType = 'freeflyer'
  packageName = 'hpp_environments'
  meshPackageName = 'hpp_environments'
  urdfName = 'ur_benchmark/pokeball'
  urdfSuffix = ""
  srdfSuffix = ""

class Ground (object):
  rootJointType = 'anchor'
  packageName = 'hpp_environments'
  urdfName = 'ur_benchmark/ground'
  meshPackageName = 'hpp_environments'
  urdfSuffix = ""
  srdfSuffix = ""

class Box (object):
    rootJointType = 'anchor'
    packageName = 'hpp_environments'
    urdfName = 'ur_benchmark/box'
    meshPackageName = 'hpp_environments'
    urdfSuffix = ""
    srdfSuffix = ""

class ConstraintGraph (Parent) :
    def __init__ (self, robot, graphName, makeGraph = True) :
        Parent.__init__ (self, robot, graphName, makeGraph)

    ## Apply constaints to a configuration
    #
    #  \param node name of the node the constraints of which to apply
    #  \param input input configuration,
    #  \retval output output configuration,
    #  \retval error norm of the residual error.
    def  applyNodeConstraints (self, node,  input) :
        return self.client.problem.applyConstraints (self.nodes [node],
                                                     input)
        
    ## Apply edge constaints to a configuration
    #
    #  \param edge name of the edge
    #  \param qfrom configuration defining the right hand side of the edge
    #         constraint,
    #  \param input input configuration,
    #  \retval output output configuration,
    #  \retval error norm of the residual error.
    #
    #  Compute a configuration in the destination node of the edge,
    #  reachable from qFrom.
    def generateTargetConfig (self, edge, qfrom, input) :
        return self.client.problem.applyConstraintsWithOffset \
            (self.edges [edge], qfrom, input)

    ## Build a path from qb to qe using the Edge::build.
    #  \param edge name of the edge to use.
    #  \param qb configuration at the beginning of the path
    #  \param qe configuration at the end of the path
    #  \retval return true if the path is built and fully projected.
    #  \retval indexNotProj -1 is the path could not be built. The index
    #                       of the built path (before projection) in the
    #                       in the ProblemSolver path vector.
    #  \retval indexProj -1 is the path could not be fully projected. The
    #                    index of the built path (before projection) in the
    #                    in the ProblemSolver path vector.
    #  No path validation is made. The paths can be retrieved using
    #  corbaserver::Problem::configAtParam
    def buildAndProjectPath (self, edge, qb, qe) :
        return self.client.problem.buildAndProjectPath \
            (self.edges [edge], qb, qe)

    ## Get error of a config with respect to a node constraint
    #
    #  \param config Configuration,
    #  \param node name of the node.
    #  \retval error the error of the node constraint for the
    #         configuration
    #  \return whether the configuration belongs to the node.
    #  Call method core::ConstraintSet::isSatisfied for the node
    #  constraints.
    def getConfigErrorForNode (self, config, nodeId) :
        return self.client.graph.getConfigErrorForNode \
          (config, self.nodes [nodeId])

    ## Add a configuration to the roadmap.
    # \param config to be added to the roadmap.
    def addConfigToRoadmap (self, config):
	return self.clientBasic.problem.addConfigToRoadmap(config)

    ## Add an edge to roadmap. If
    # \param config1, config2 the ends of the path,
    # \param pathId the index if the path in the vector of path,
    # \param bothEdges if FALSE, only add config1 to config2, otherwise, If TRUE. add edges config1->config2 AND config2->config1.
    def addEdgeToRoadmap (self, config1, config2, pathId, bothEdges):
	return self.clientBasic.problem.addEdgeToRoadmap \
          (config1, config2, pathId, bothEdges)

    ## Set that an edge is short
    #  \param edge name of the edge
    #  \param True or False
    #
    #  When an edge is tagged as short, extension along this edge is
    #  done differently in RRT-like algorithms. Instead of projecting
    #  a random configuration in the destination node, the
    #  configuration to extend itself is projected in the destination
    #  node. This makes the rate of success higher.
    def setShort (self, edge, isShort) :
      return self.client.graph.setShort (self.edges [edge], isShort)


robot = Robot ('ur5-pokeball', 'ur5')
robot.client.basic.problem.setErrorThreshold (1e-4)
robot.client.basic.problem.setMaxIterations (40)

ps = ProblemSolver (robot)
vf = ViewerFactory (ps)
gripperName = 'ur5/wrist_3_link-tool0_fixed_joint'
ballName = 'pokeball/base_joint_SO3'
