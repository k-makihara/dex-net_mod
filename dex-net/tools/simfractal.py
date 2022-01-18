import pybullet as p
import time
import math
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

# time.sleep(20)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setAdditionalSearchPath("../urdf")
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "visualShapeBench.json")
#useMaximalCoordinates is much faster then the default reduced coordinates (Featherstone)

p.loadURDF("plane100.urdf", useMaximalCoordinates=True)

# time.sleep(10)
#disable rendering during creation.
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#disable tinyrenderer, software (CPU) renderer, we don't use it here
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

shift = [0, -0.25, 0]
meshScale = [1, 1, 1]
collisionmeshScale = [1, 1, 1]
#the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/test.obj",
                                    rgbaColor=[0, 0, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName="/home/user/Downloads/IROS2022_dex-fractal-20211224T053701Z-001/IROS2022_dex-fractal/3dfractal_render_modern_opengl/3dfractal_render_modern_opengl/testply/test_vhacd.obj",
                                          collisionFramePosition=shift,
                                          meshScale=meshScale)

test = p.createMultiBody(baseMass=0.12,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0, 0, 0.5],
                      useMaximalCoordinates=True)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)


# Camera Setting
width = 640
height = 480

fov = 45
aspect = width / height
near = 2.5
far = 5.0
view_matrix = p.computeViewMatrix([0, 0, 4.6], [0, 0, 0], [1, 0, 0])
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# Get depth values using the OpenGL renderer
images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
depth_buffer_opengl = np.reshape(images[3], [height, width])
depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
seg_opengl = np.reshape(images[4], [height, width]) * 1. / 255.
# time.sleep(2)

# Get depth values using Tiny renderer
images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_TINY_RENDERER)
depth_buffer_tiny = np.reshape(images[3], [height, width])
depth_tiny = far * near / (far - (far - near) * depth_buffer_tiny)
rgb_tiny = np.reshape(images[2], (height, width, 4)) * 1. / 255.
seg_tiny = np.reshape(images[4], [height, width]) * 1. / 255.


# Plot both images - should show depth values of 0.45 over the cube and 0.5 over the plane
plt.subplot(3, 2, 1)
plt.imshow(depth_opengl, cmap='jet')
plt.title('Depth OpenGL3')

plt.subplot(3, 2, 2)
plt.imshow(depth_tiny, cmap='jet')
plt.title('Depth TinyRenderer')

plt.subplot(3, 2, 3)
plt.imshow(rgb_opengl)
plt.title('RGB OpenGL3')

plt.subplot(3, 2, 4)
plt.imshow(rgb_tiny)
plt.title('RGB Tiny')

plt.subplot(3, 2, 5)
plt.imshow(seg_opengl)
plt.title('Seg OpenGL3')

plt.subplot(3, 2, 6)
plt.imshow(seg_tiny)
plt.title('Seg Tiny')

plt.subplots_adjust(hspace=0.7)

plt.show()




while (1):
  time.sleep(1./240.)
