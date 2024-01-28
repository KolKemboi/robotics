import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
bot = p.loadURDF("franka_panda/panda.urdf", startPos, startOrientation, useFixedBase = True)

p.setGravity(0, 0, -9.8)

jointsIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations = 10)
p.changeDynamics(bot, -1, linearDamping = 0, angularDamping = 0)

for joint in range(p.getNumJoints(bot)):
    p.changeDynamics(bot, joint, linearDamping = 0, angularDamping=0)
    info = p.getJointInfo(bot, joint)
    jointName = info[1]
    jointType = info[2]

    ##prismatic Joints have a value of 1
    ##revolute Joints have a value of 0
    if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
        jointsIds.append(joint)
        paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, 0))


p.setRealTimeSimulation(1)
for i in range(100_000):
    p.stepSimulation()
    for idx, joint in enumerate(paramIds):
        c = paramIds[idx]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(bot, jointsIds[idx], p.POSITION_CONTROL, targetPos, force = 5 * 240)
    time.sleep(1.0/240.)


p.disconnect()