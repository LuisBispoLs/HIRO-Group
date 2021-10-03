import os
import math
import pybullet as pyb
import pybullet_data as pyb_d

pyb.connect(pyb.GUI)

urdfPath = pyb_d.getDataPath()

Panda = pyb.loadURDF(os.path.join(urdfPath, "franka_panda/panda.urdf"), useFixedBase = True)

Table = pyb.loadURDF(os.path.join(urdfPath, "table/table.urdf"), basePosition = [0.5,0.,-0.65])

Table2 = pyb.loadURDF(os.path.join(urdfPath, "table/table.urdf"), basePosition = [-0.5,0.,-0.65])

Basket = pyb.loadURDF(os.path.join(urdfPath, "tray/traybox.urdf"), basePosition = [0.65,-0.1,0])

Basket2 = pyb.loadURDF(os.path.join(urdfPath, "tray/traybox.urdf"), basePosition = [-0.65,-0.1,0])

Object = pyb.loadURDF(os.path.join(urdfPath, "random_urdfs/000/000.urdf"), basePosition = [0.7,0,0.1])

#obj = pyb.loadURDF(os.path.join(urdfPath, "jenga/jenga.urdf"), basePosition=[0.7,0,0.1])


pyb.setGravity(0,0,-9.81)



pyb.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])


state_durations = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
control_dt = 1./120.
pyb.setTimestep = control_dt
state_t = 0.
current_state = 0

while True:
    state_t += control_dt
    pyb.configureDebugVisualizer(pyb.COV_ENABLE_SINGLE_STEP_RENDERING) 


    if current_state == 0:


        #   pybullet.setJointMotorControl2([objectUid],[jointIndex],[controller],[targetPosition]) 
        #       will control the robot joints and move them.


        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.)
        pyb.setJointMotorControl2(Panda, 2, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.)
        pyb.setJointMotorControl2(Panda, 4, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 5, 
                        pyb.POSITION_CONTROL,3*math.pi/4)
        pyb.setJointMotorControl2(Panda, 6, 
                        pyb.POSITION_CONTROL,-math.pi/4.)
        pyb.setJointMotorControl2(Panda, 9, 
                        pyb.POSITION_CONTROL, 0.08)
        pyb.setJointMotorControl2(Panda, 10, 
                        pyb.POSITION_CONTROL, 0.08)

    if current_state == 1:
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.+.15)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.+.15)

    if current_state == 2:
        pyb.setJointMotorControl2(Panda, 9, 
                        pyb.POSITION_CONTROL, 0.0, force = 200)
        pyb.setJointMotorControl2(Panda, 10, 
                        pyb.POSITION_CONTROL, 0.0, force = 200)
           
    if current_state == 3:        
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.-1)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.-1)

    if current_state == 4:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi, force = 20)

    if current_state == 5:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.)
        pyb.setJointMotorControl2(Panda, 2, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.)
        pyb.setJointMotorControl2(Panda, 4, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 5, 
                        pyb.POSITION_CONTROL,3*math.pi/4)
        pyb.setJointMotorControl2(Panda, 6, 
                        pyb.POSITION_CONTROL,-math.pi/4.)
        pyb.setJointMotorControl2(Panda, 9, 
                        pyb.POSITION_CONTROL, 0.08)
        pyb.setJointMotorControl2(Panda, 10, 
                        pyb.POSITION_CONTROL, 0.08)

    if current_state == 6:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi)
        

    if current_state == 7:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL, -math.pi)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 2, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 4, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 5, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 6, 
                        pyb.POSITION_CONTROL,0)
        
    if current_state == 8:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL, math.pi)

    if current_state == 9:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL, -math.pi)

    if current_state == 10:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.)
        pyb.setJointMotorControl2(Panda, 2, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.)
        pyb.setJointMotorControl2(Panda, 4, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 5, 
                        pyb.POSITION_CONTROL,3*math.pi/4)
        pyb.setJointMotorControl2(Panda, 6, 
                        pyb.POSITION_CONTROL,-math.pi/4.)
        pyb.setJointMotorControl2(Panda, 9, 
                        pyb.POSITION_CONTROL, 0.08)
        pyb.setJointMotorControl2(Panda, 10, 
                        pyb.POSITION_CONTROL, 0.08)

    if current_state == 11:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.+.15)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.+.15)

    if current_state == 12:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi)
        pyb.setJointMotorControl2(Panda, 9, 
                        pyb.POSITION_CONTROL, 0.0, force = 200)
        pyb.setJointMotorControl2(Panda, 10, 
                        pyb.POSITION_CONTROL, 0.0, force = 200)

    if current_state == 13:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,-math.pi)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.-1)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.-1)

    if current_state == 14:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,math.pi, force = 20)

    if current_state == 15:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,math.pi/4.)
        pyb.setJointMotorControl2(Panda, 2, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,-math.pi/2.)
        pyb.setJointMotorControl2(Panda, 4, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 5, 
                        pyb.POSITION_CONTROL,3*math.pi/4)
        pyb.setJointMotorControl2(Panda, 6, 
                        pyb.POSITION_CONTROL,-math.pi/4.)
        pyb.setJointMotorControl2(Panda, 9, 
                        pyb.POSITION_CONTROL, 0.08)
        pyb.setJointMotorControl2(Panda, 10, 
                        pyb.POSITION_CONTROL, 0.08)

    if current_state == 16:
        pyb.setJointMotorControl2(Panda, 0, 
                        pyb.POSITION_CONTROL, -math.pi)
        pyb.setJointMotorControl2(Panda, 1, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 2, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 3, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 4, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 5, 
                        pyb.POSITION_CONTROL,0)
        pyb.setJointMotorControl2(Panda, 6, 
                        pyb.POSITION_CONTROL,0)




    if state_t >state_durations[current_state]:
        current_state += 1
        if current_state >= len(state_durations):
            current_state = 0
        state_t = 0




    pyb.stepSimulation()      # runs one step of the simulation.