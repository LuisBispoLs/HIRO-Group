import os                               # creating path for finding the objects that we want to add to the environment.
import math, time
import pybullet as p
import pybullet_data as p_d             # import objects and available components in pullet
import pytinydiffsim as dp
from random import uniform as rand


# ========================================================================
# ========================================================================
# ======================        SET PYBULLET        ======================

p.connect(p.GUI)                    #  Simulation with graphical user interface
#p.connect(p.DIRECT)                 #  Simulation without GUI

p.setGravity(0,0,-9.81)               # Values differents of zero in the 1st and 2nd arguments can be used to simulate wind

urdfPath = p_d.getDataPath()
Plane = p.loadURDF(os.path.join(urdfPath, "plane.urdf"))
Panda = p.loadURDF(os.path.join(urdfPath, "franka_panda/panda.urdf"), useFixedBase = True)
Basket1 = p.loadURDF(os.path.join(urdfPath, "tray/traybox.urdf"), basePosition = [ 0.65 , 0 , 0 ])
Basket2 = p.loadURDF(os.path.join(urdfPath, "tray/traybox.urdf"), basePosition = [0.65, 7., 0])
Object = p.loadURDF(os.path.join(urdfPath, "random_urdfs/000/000.urdf"), basePosition = [ 0.7 , 0 , 0.1 ])
#Object = p.loadURDF(os.path.join(urdfPath, "jenga/jenga.urdf"), basePosition=[0.6,0,0.1])


#       CAMERA:
#p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=90, cameraPitch=-45, cameraTargetPosition=[1.5,5,0.4])


# ========================================================================
# ========================================================================
# ===================        PYBULLET SIMULATION        ==================

state_durations = [1,1,1,1,1,1]
control_dt = 1./240.
p.setTimeStep(control_dt)
state_t = 0
current_state = 0

Force_variation = rand( 25. , 1000. )        # It has a random value

while True:
    state_t += control_dt
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) 

    if current_state == 0:
        #   pullet.setJointMotorControl2([objectUid],[jointIndex],[controller],[targetPosition]) 
        #       will control the robot joints and move them.
        p.setJointMotorControl2(Panda, 0, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 1, 
                        p.POSITION_CONTROL,math.pi/4.)
        p.setJointMotorControl2(Panda, 2, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 3, 
                        p.POSITION_CONTROL,-math.pi/2.)
        p.setJointMotorControl2(Panda, 4, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 5, 
                        p.POSITION_CONTROL,3*math.pi/4.)
        p.setJointMotorControl2(Panda, 6, 
                        p.POSITION_CONTROL,-math.pi/4.)
        p.setJointMotorControl2(Panda, 9, 
                        p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(Panda, 10, 
                        p.POSITION_CONTROL, 0.08)
 
    if current_state == 1:
        p.setJointMotorControl2(Panda, 1, 
                        p.POSITION_CONTROL,math.pi/4.+.15)
        p.setJointMotorControl2(Panda, 3, 
                        p.POSITION_CONTROL,-math.pi/2.+.15)
          
    if current_state == 2:
        p.setJointMotorControl2(Panda, 9, 
                        p.POSITION_CONTROL, 0.0, force = 200)
        p.setJointMotorControl2(Panda, 10, 
                        p.POSITION_CONTROL, 0.0, force = 200)
           
    if current_state == 3:        
        p.setJointMotorControl2(Panda, 1, 
                                p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 3, 
                                p.POSITION_CONTROL,-math.pi/2.)
        p.setJointMotorControl2(Panda, 5, 
                                p.POSITION_CONTROL, math.pi/2.)

    if current_state == 4:
        p.setJointMotorControl2(Panda, 4, 
                        p.POSITION_CONTROL, math.pi, force = Force_variation)      # 50    |   25(1.5m) <> 1000(10m)
      
    if current_state == 5:
        p.setJointMotorControl2(Panda, 0, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 1, 
                        p.POSITION_CONTROL,0.)
        p.setJointMotorControl2(Panda, 2, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 3, 
                        p.POSITION_CONTROL,0.)
        p.setJointMotorControl2(Panda, 4, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 5, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 6, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(Panda, 9, 
                        p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(Panda, 10, 
                        p.POSITION_CONTROL, 0.08)

    
    if state_t > state_durations[current_state] :
        current_state += 1

        if current_state >= len(state_durations):
            current_state = 0

            #   Varying the throw force:
            print('\nForce: ', Force_variation)
            Force_variation = rand( 25. , 1000. )        # It has a random value

            #   Getting the location of the thrown object:
            Object_location = p.getBasePositionAndOrientation(Object)
            print('Object location: {0[0]}' .format(Object_location))

            #   Removing the thrown object and the 2nd basket from the world:
            p.removeBody(Object)
            p.removeBody(Basket2)

            #   Object appears again in the basket:
            Object = p.loadURDF(os.path.join(urdfPath, "random_urdfs/000/000.urdf"),
                                            basePosition = [ 0.7 , 0 , 0.1 ]) 

            #   RANDOM 2ND BASKET LOCATION
            Basket2_location = [0.65, rand(1.5,11), 0]        # vector with x, y, z basket location; y axis has a random value
            Basket2 = p.loadURDF(os.path.join(urdfPath, "tray/traybox.urdf"),
                                basePosition = Basket2_location)

            print('2nd basket location: ', Basket2_location)

        state_t = 0
    
    p.stepSimulation()      # runs one step of the simulation.
    #p.resetSimulation()
