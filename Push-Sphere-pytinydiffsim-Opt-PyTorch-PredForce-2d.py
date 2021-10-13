from absl import app
from time import sleep
from random import uniform as rand

import math
import torch
from torch.autograd import Variable

import pytinyopengl3 as gl
import pytinydiffsim as tds

def push(force, friction, step, render, tensor):

    if render:
        app.renderer.remove_all_instances()

    bodies = []
    visuals = []

    #   BUILD WORLD:        ========================================
    world = tds.TinyWorld()
    world.gravity = tds.Vector3(0, 0, -9.81)
    world.friction = friction
    
    if tensor:
        force_x_tensor = force[0].tolist()
        force_y_tensor = force[1].tolist()
        print("|")
        print("| force_pred_x:", force_x_tensor)
        print("| force_pred_y:", force_y_tensor)
        force_components = tds.Vector3(force_x_tensor, force_y_tensor, 0)
    else:
        force_components = tds.Vector3(force[0], force[1], 0)


    #   PLANE CONF.:        ========================================
    pos_plane = tds.Vector3(0, 0, 0)
    mass_plane = 0.0

    plane = tds.TinyPlane()
    plane_body = tds.TinyRigidBody(mass_plane, plane)
    plane_body.world_pose.position = pos_plane
    bodies.append(plane_body)
    
    #   RENDERING THE PLANE    ========================================
    width = 256
    height = 256
    pixels = [255] * width * height * 3

    if render:
        orn = gl.TinyQuaternionf(0, 0, 0, 1)
        pos = gl.TinyVector3f(tds.get_debug_double(plane_body.world_pose.position[0]),
                            tds.get_debug_double(plane_body.world_pose.position[1]),
                            tds.get_debug_double(plane_body.world_pose.position[2]))
        color = gl.TinyVector3f(0,0,0)
        opacity = 0.2
        scaling = gl.TinyVector3f(tds.get_debug_double(1.),
                                tds.get_debug_double(1.),
                                tds.get_debug_double(1.))
        textureIndex = app.renderer.register_texture(pixels, width, height, False)
        shape = app.register_cube_shape(10, 10, 0.01, textureIndex, 40)
        plane_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity)
        visuals.append(plane_id)
    
    
    #   SPHERE CONF.:       ========================================
    initial_pos = tds.Vector3(0, 0, radius+0.01)

    sphere = tds.TinySphere(radius)
    sphere_body = tds.TinyRigidBody(mass_sphere, sphere)
    sphere_body.world_pose.position = initial_pos
    bodies.append(sphere_body)
    sphere_body.apply_central_force(force_components)

    #   RENDERING THE SPHERE        ========================================
    if render:
        orn = gl.TinyQuaternionf(0, 0, 0, 1)
        pos = gl.TinyVector3f(tds.get_debug_double(sphere_body.world_pose.position[0]),
                            tds.get_debug_double(sphere_body.world_pose.position[1]),
                            tds.get_debug_double(sphere_body.world_pose.position[2]))
        color = gl.TinyVector3f(0,0,1)       # BLUE and BLACK
        opacity = 1
        scaling = gl.TinyVector3f(tds.get_debug_double(radius),
                                tds.get_debug_double(radius),
                                tds.get_debug_double(radius))
        textureIndex = -1
        shape = app.register_graphics_unit_sphere_shape(gl.EnumSphereLevelOfDetail.SPHERE_LOD_HIGH, textureIndex)
        sphere_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity)
        visuals.append(sphere_id)


    rb_solver = tds.TinyConstraintSolver()

    if render:
        app.renderer.write_transforms()
    #world.step(dt)

    for iter in range(step):
        sphere_body.apply_gravity(world.gravity)
        sphere_body.apply_force_impulse(dt)
        sphere_body.clear_forces()

        # COLLISION DETECTION:        ========================================
        dispatcher = world.get_collision_dispatcher()
        contacts = world.compute_contacts_rigid_body(bodies,dispatcher)
        #print("contacts=",contacts)

        # COLLISION SOLVER:        ========================================
        num_solver_iterations = 50

        for solver_iter in range(num_solver_iterations):
            for c in contacts:
                rb_solver.resolve_collision(c,dt)

        for b in bodies:
            b.integrate(dt)

        #print("step ", iter,">>> sphere_body.linear_velocity = ", sphere_body.linear_velocity)


        # sync visual transforms
        if render:
            for v in range (len(bodies)):
                b = bodies[v]
                pos = gl.TinyVector3f(tds.get_debug_double(b.world_pose.position[0]),
                                    tds.get_debug_double(b.world_pose.position[1]),
                                    tds.get_debug_double(b.world_pose.position[2]))
                #print("pos=",pos)
                orn = gl.TinyQuaternionf(0,0,0,1)
                app.renderer.write_single_instance_transform_to_cpu(pos, orn, visuals[v])
                
            app.renderer.update_camera(2)
            dg = gl.DrawGridData()
            dg.drawAxis = True
            app.draw_grid(dg)

            #   INDICATING THE TARGET:      ========================================
            from_line = gl.TinyVector3f(target[0], target[1], radius)
            to_line = gl.TinyVector3f(target[0], target[1], 4)
            color = gl.TinyVector3f(1,0,0)
            width = 3
            app.draw_text_3d("Target", target[0], target[1], 4, 1)
            app.renderer.draw_line(from_line,to_line,color,width)
            
            app.renderer.write_transforms()
            app.renderer.render_scene()
            app.swap_buffer()
      

    print("| Sphere position:", sphere_body.world_pose.position)
    if math.isnan(sphere_body.world_pose.position[0]):
        print("\n---Simulation Failed---\n")
        quit()
    
    if tensor:
        S_pos_0, S_pos_1, S_pos_2 = sphere_body.world_pose.position[0], sphere_body.world_pose.position[1], sphere_body.world_pose.position[2]
        S_pos = [S_pos_0, S_pos_1, S_pos_2]
        S_pos_convert = Variable(torch.tensor([S_pos, [1,1,1], [1,1,1]]).type(torch.FloatTensor), requires_grad=False)
        #S_pos_convert = Variable(torch.tensor([S_pos, [0,0,0], [0,0,0]]).type(torch.FloatTensor), requires_grad=False)
        return S_pos_convert, S_pos
    else:
        return sphere_body.world_pose.position

def Optimization(friction, target, N_interactions_Opt, steps_sim, learning_rate):
    dtype = torch.FloatTensor

    target_Tensor = torch.tensor(target).type(dtype)
    #print('\ntarget_Tensor:', target_Tensor)

    predicted_force = Variable(torch.randn(3).type(dtype), requires_grad=True)
    #print('\npredicted_force:', predicted_force)
    #print('\ntest:',torch.diag(predicted_force))

    #optim = torch.optim.Adam([predicted_force], lr=learning_rate)
    optim = torch.optim.RMSprop([predicted_force], lr=learning_rate)

    for i in range(N_interactions_Opt):

        _, simulated_target = push(predicted_force, friction, steps_sim, render=False, tensor=True)
        simulated_target_Tensor = torch.tensor([simulated_target]).type(dtype)

        put_grad_fn = torch.matmul(torch.diag(predicted_force), simulated_target_Tensor.t())
        #print('\nput_grad_fn',put_grad_fn)

        let_only_grad_fn = torch.div(put_grad_fn, predicted_force.data)
        #print('\nlet_only_grad_fn:', let_only_grad_fn)

        sim_target_with_grad_fn = torch.diag(let_only_grad_fn, 0)
        #print('\nsim_target_with_grad_fn:', sim_target_with_grad_fn)

        optim.zero_grad()
        loss = torch.nn.MSELoss()(target_Tensor, sim_target_with_grad_fn)
        print("|  >>> Interaction #", i, " >>>  Loss:", float(loss.data))

        loss.backward()
        optim.step()

    #print('\npredicted_force:', predicted_force, '\n')
    predicted_force_x = predicted_force[0].tolist()
    predicted_force_y = predicted_force[1].tolist()
    return [predicted_force_x, predicted_force_y, 0]

"""
# =======================================================
# =======================================================
# =================     SETUP:     ======================
'''
mass_sphere = 1.0
radius = tds.fraction(1, 2)
target = [8, 5, 0]
friction = tds.fraction(5, 10)
'''
mass_sphere = rand(0.1, 10.)
radius = rand(0.2, 2.)
target = [rand(-9., 9.), rand(-9., 9.), 0]
friction = rand(.1, .99)
#'''

#   Simulation:
steps = 300
dt = tds.fraction(1, 60)
render = True
if render:
    app = gl.TinyOpenGL3App("Visualizer")
    app.renderer.init()
    cam = gl.TinyCamera()
    cam.set_camera_distance(15)
    #cam.set_camera_roll(15.)
    cam.set_camera_pitch(-45)
    #cam.set_camera_target_position(0,0,0)
    #cam.set_camera_up_axis()
    #cam.set_camera_up_vector(,,)
    #cam.set_camera_yaw(-30)
    cam.set_camera_yaw(170)
    app.renderer.set_camera(cam)

#   Optimization:
N_interactions = 500
#learning_rate = 1e-3
learning_rate = 10
# =======================================================
# =======================================================
# =======================================================

#   RUNNING:
    # Push before optimization
print("---Seed Simulation---")
push([0,0,0], friction, steps, render, tensor=False)

    # Optimization
print("\n---Optimization---")
#force_x_opt = Optimization(target, N_interactions, steps, learning_rate)
force_opt = Optimization(friction, target, N_interactions, steps, learning_rate)

    # Push after optimization
print("\n---Final Simulation---")
print(  "\n| Mass:", mass_sphere,
        "\n| Radius:", radius,
        "\n| Friction:", friction,
        "\n| Target:", target)
push(force_opt, friction, steps, render, tensor=False)
sleep(5)
"""


#"""
#   Simulation:
steps = 300
dt = tds.fraction(1, 60)
render = True
if render:
    app = gl.TinyOpenGL3App("Visualizer")
    app.renderer.init()
    cam = gl.TinyCamera()
    cam.set_camera_distance(15)
    #cam.set_camera_roll(15.)
    cam.set_camera_pitch(-45)
    #cam.set_camera_target_position(0,0,0)
    #cam.set_camera_up_axis()
    #cam.set_camera_up_vector(,,)
    #cam.set_camera_yaw(-30)
    cam.set_camera_yaw(170)
    app.renderer.set_camera(cam)

#   Optimization:
N_interactions = 500
#learning_rate = 1e-3
learning_rate = 10

while 1:

    # =======================================================
    # =======================================================
    # =================     SETUP:     ======================
    '''
    mass_sphere = 1.0
    radius = tds.fraction(2, 2)
    target = [8, 4, 0]
    friction = tds.fraction(5, 10)
    '''
    mass_sphere = rand(0.1, 10.)
    radius = rand(0.2, 2.)
    target = [rand(-9., 9.), rand(-9., 9.), 0]
    friction = rand(.1, .99)
    #'''

    # =======================================================
    # =======================================================
    # =======================================================

    #   RUNNING:
        # Optimization
    print("\n---Optimization---")
    #force_x_opt = Optimization(target, N_interactions, steps, learning_rate)
    force_opt = Optimization(friction, target, N_interactions, steps, learning_rate)

        # Push after optimization
    print("\n---Final Simulation---")
    print(  "\n| Mass:", mass_sphere,
            "\n| Radius:", radius,
            "\n| Friction:", friction,
            "\n| Target:", target)
    push(force_opt, friction, steps, render, tensor=False)
    sleep(5)

#"""