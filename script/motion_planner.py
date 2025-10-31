class MotionPlanner:
    def __init__(self, robot, ps):
        self.robot = robot
        self.ps = ps

    def solveBiRRT(self, maxIter=float("inf")):
        # print("Method solveBiRRT is not implemented yet")
        self.ps.prepareSolveStepByStep()
        finished = False

        # In the framework of the course,
        # we restrict ourselves to 2 connected components.
        nbCC = self.ps.numberConnectedComponents()
        if nbCC != 2:
            raise Exception("There should be 2 connected components.")

        iter = 0
        while True:
            # RRT begin
            # write your algorithm here
            # RRT end
            random_config = self.robot.shootRandomConfig()

            c1, _ = self.ps.getNearestConfig(random_config, 0)
            c2, _ = self.ps.getNearestConfig(random_config, 1)

            _, path1, _ = self.ps.directPath(c1, random_config, True)
            _, path2, _ = self.ps.directPath(c2, random_config, True)

            length1 = self.ps.pathLength(path1)
            length2 = self.ps.pathLength(path2)

            config1 = self.ps.configAtParam(path1, length1)
            config2 = self.ps.configAtParam(path2, length2)

            self.ps.addConfigToRoadmap(config1)
            self.ps.addConfigToRoadmap(config2)

            self.ps.addEdgeToRoadmap(c1, config1, path1, True)
            self.ps.addEdgeToRoadmap(c2, config2, path2, True)
            # Check if the problem is solved.
            nbCC = self.ps.numberConnectedComponents()
            if nbCC == 1:
                # Problem solved
                finished = True
                break
            iter = iter + 1
            if iter > maxIter:
                break
        if finished:
            self.ps.finishSolveStepByStep()
            return self.ps.numberPaths() - 1

    def solvePRM(self):
        self.ps.prepareSolveStepByStep()
        # PRM begin
        # PRM end
        self.ps.finishSolveStepByStep()
