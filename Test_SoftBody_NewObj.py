# Refer Source of below test: https://github.com/bulletphysics/bullet3/blob/6a59241074720e9df119f2f86bc01765917feb1e/examples/pybullet/examples/deformable_ball.py 
import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)   # convers the simulation to Finite Element (FEM) Based from the default Position Based Dynamics(PBD)

p.setGravity(0, 0, -10)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,0],planeOrn)

boxId = p.loadURDF("cube.urdf", [0,3,2],useMaximalCoordinates = True)

# BALL OBJECT LIKE IN PYB EXAMPLES
# ball_s_ID = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition = [0,-1.5,1], scale = 0.15, mass = 1.,
#                            useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=20,
#                            springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 0,
#                            frictionCoeff = .5, useFaceContact=1, collisionMargin = 0.04)
spinnerId = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition = [0,0,1], scale = 0.25, mass = 1,
                        useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = 600, NeoHookeanDamping = 0.05,
                        useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001, useFaceContact=1)

# TRY NEW OBJECT                  
# ball_s_ID = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition = [0,-2.5,1], scale = 3, mass = 1.,
#                            useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=20,
#                            springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 0,
#                            frictionCoeff = .5, useFaceContact=1, collisionMargin = 0.04)
spinner_s_Id = p.loadSoftBody("Spinner_Press.obj", simFileName = "Spinner_Press.vtk", basePosition = [0,0,1], scale = 0.25, mass = 1,
                        useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = 600, NeoHookeanDamping = 0.05,
                        useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001, useFaceContact=1)

# p.setTimeStep(0.001)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")
p.setRealTimeSimulation(1)
# p.setTimeStep(0.001)

while p.isConnected():

  p.stepSimulation()  
  p.setGravity(0,0,-10)

  #there can be some artifacts in the visualizer window, 
  #due to reading of deformable vertices in the renderer,
  #while the simulators updates the same vertices
  #it can be avoided using
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #but then things go slower
  # sleep(1./240.)
  
#p.resetSimulation()
#p.stopStateLogging(logId)
