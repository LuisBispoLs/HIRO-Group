from absl import app
from time import sleep
from random import uniform as rand

import torch
from torch.autograd import Variable

import pytinyopengl3 as gl
import pytinydiffsim as ds

#def push(force_x, force_y, step, render):
def push(force_x, step, render):

    if render:
        app.renderer.remove_all_instances()

    bodies = []
    visuals = []

    #   BUILD WORLD:        ========================================
    world = ds.TinyWorld()
    world.gravity = ds.Vector3(0, 0, -9.81)
    #world.friction = ds.fraction(5, 10)
    #world.friction = ds.fraction(3, 10)
    #world.friction = ds.fraction(1, 1)
    #test = world.friction
    #print("\nFriction = ", test)
    
    force_components = ds.Vector3(force_x, 0, 0)


    #   PLANE CONF.:        ========================================
    pos_plane = ds.Vector3(0, 0, 0)
    mass_plane = 0.0

    plane = ds.TinyPlane()
    plane_body = ds.TinyRigidBody(mass_plane, plane)
    plane_body.world_pose.position = pos_plane
    bodies.append(plane_body)
    
    #   RENDERING THE PLANE    ========================================
    width = 256
    height = 256
    pixels = [255] * width * height * 3

    if render:
        orn = gl.TinyQuaternionf(0, 0, 0, 1)
        pos = gl.TinyVector3f(ds.get_debug_double(plane_body.world_pose.position[0]),
                            ds.get_debug_double(plane_body.world_pose.position[1]),
                            ds.get_debug_double(plane_body.world_pose.position[2]))
        #color = gl.TinyVector3f(0,0,1)       # BLUE and BLACK
        #color = gl.TinyVector3f(1,0,0)       # RED and BLACK
        #color = gl.TinyVector3f(0,1,0)       # GREEN
        #color = gl.TinyVector3f(1,1,0)       # YELLOW
        color = gl.TinyVector3f(0,0,0)
        opacity = 0.2
        scaling = gl.TinyVector3f(ds.get_debug_double(1.),
                                ds.get_debug_double(1.),
                                ds.get_debug_double(1.))
        textureIndex = app.renderer.register_texture(pixels, width, height, False)
        shape = app.register_cube_shape(10, 10, 0.01, textureIndex, 40)
        plane_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity)
        visuals.append(plane_id)
    
    
    #   SPHERE CONF.:       ========================================
    initial_pos = ds.Vector3(0, 0, radius+0.01)

    sphere = ds.TinySphere(radius)
    sphere_body = ds.TinyRigidBody(mass_sphere, sphere)
    sphere_body.world_pose.position = initial_pos
    bodies.append(sphere_body)
    sphere_body.apply_central_force(force_components)

    #   RENDERING THE SPHERE        ========================================
    if render:
        orn = gl.TinyQuaternionf(0, 0, 0, 1)
        pos = gl.TinyVector3f(ds.get_debug_double(sphere_body.world_pose.position[0]),
                            ds.get_debug_double(sphere_body.world_pose.position[1]),
                            ds.get_debug_double(sphere_body.world_pose.position[2]))
        color = gl.TinyVector3f(0,0,1)       # BLUE and BLACK
        #color = gl.TinyVector3f(1,0,0)       # RED and BLACK
        #color = gl.TinyVector3f(0,1,0)       # GREEN
        opacity = 1
        scaling = gl.TinyVector3f(ds.get_debug_double(radius),
                                ds.get_debug_double(radius),
                                ds.get_debug_double(radius))
        textureIndex = -1
        shape = app.register_graphics_unit_sphere_shape(gl.EnumSphereLevelOfDetail.SPHERE_LOD_HIGH, textureIndex)
        sphere_id = app.renderer.register_graphics_instance(shape, pos, orn, color, scaling, opacity)
        visuals.append(sphere_id)


    rb_solver = ds.TinyConstraintSolver()

    if render:
        app.renderer.write_transforms()
    #world.step(dt)

    for iter in range(step):
        '''
        for b in bodies:
            b.apply_gravity(world.gravity)
            b.apply_force_impulse(dt)
            b.clear_forces()
        '''
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
                #print("v=",v)
                b = bodies[v]
                pos = gl.TinyVector3f(ds.get_debug_double(b.world_pose.position[0]),
                                    ds.get_debug_double(b.world_pose.position[1]),
                                    ds.get_debug_double(b.world_pose.position[2]))
                #print("pos=",pos)
                orn = gl.TinyQuaternionf(0,0,0,1)
                app.renderer.write_single_instance_transform_to_cpu(pos, orn, visuals[v])
                
            app.renderer.update_camera(2)
            dg = gl.DrawGridData()
            #dg.drawAxis = True
            app.draw_grid(dg)

            #   INDICATING THE TARGET:      ========================================
            from_line = gl.TinyVector3f(target[0], target[1], 0.2)
            to_line = gl.TinyVector3f(target[0], target[1], 4)
            color = gl.TinyVector3f(1,0,0)
            width = 3
            app.draw_text_3d("Target", target[0], target[1], 4, 1)
            app.renderer.draw_line(from_line,to_line,color,width)
            
            app.renderer.write_transforms()
            app.renderer.render_scene()
            app.swap_buffer()
      
    

    #   LOSS FUNCTION:      ========================================
    #loss = (sphere_body.world_pose.position[0] - target[0]).pow(2)      # (final_sphere_pos_x - target_x)^2
    '''
    print("\nForce X = ", force_components[0])
    print("Sphere position = ", sphere_body.world_pose.position)
    print("Target = ", target)
    print("Loss = ", loss)
    '''
    #print("Sphere position = ", sphere_body.world_pose.position, "Target = ", target)

    print("| Sphere X position:", sphere_body.world_pose.position[0])
    return sphere_body.world_pose.position[0]


def Optimization(target, N_interactions_Opt, steps_sim, learning_rate):
    dtype = torch.FloatTensor

    # N is batch size; D_in is input dimension;
    # H is hidden dimension; D_out is output dimension.
    N, D_in, H, D_out = 1, 3, 30, 3

    target_Tensor = Variable(torch.tensor([target, [0,0,0]]).type(dtype), requires_grad=False)
    #y = Variable(torch.randn(N, D_out).type(dtype), requires_grad=False)

    # Weights:
    w1 = Variable(torch.randn(D_in, H).type(dtype), requires_grad=True)
    w2 = Variable(torch.randn(H, D_out).type(dtype), requires_grad=True)

    for i in range(N_interactions_Opt):

        # Neural Network:
        force_pred = target_Tensor.mm(w1).clamp(min=0).mm(w2)
        force_pred_x = force_pred[0,0].tolist()
        print("| force_pred_x:",force_pred_x)

        simulated_target = push(force_pred_x, steps_sim, render=False)

        loss = (simulated_target - target[0])**2#.pow(2).sum()      # (final_sphere_pos_x - target_x)^2
        print("  #", i, ":", loss)

        loss.backward()

        # Update weights using gradient descent; w1.data and w2.data are Tensors,
        # w1.grad and w2.grad are Variables and w1.grad.data and w2.grad.data are
        # Tensors.
        w1.data -= learning_rate * w1.grad.data
        w2.data -= learning_rate * w2.grad.data

        # Manually zero the gradients 
        w1.grad.data.zero_()
        w2.grad.data.zero_()

    return force_pred_x  


# =======================================================
# =======================================================
# =================     SETUP:     ======================
mass_sphere = 2.0
radius = ds.fraction(1, 2)
target = [7, 0, 0]

#radius = rand(0.2, 2.)
#mass_sphere = rand(0.1, 5.)
#target = ds.Vector3( rand(-8., 8.), 0, 0)


#   Simulation:
steps = 300
dt = ds.fraction(1, 60)
render = False
if render:
    app = gl.TinyOpenGL3App("Visualizer")
    app.renderer.init()
    cam = gl.TinyCamera()
    cam.set_camera_distance(10)
    #cam.set_camera_roll(15.)
    cam.set_camera_pitch(-40)
    #cam.set_camera_target_position(0,0,0)
    #cam.set_camera_up_axis()
    #cam.set_camera_up_vector(,,)
    #cam.set_camera_yaw(-30)
    cam.set_camera_yaw(0)
    app.renderer.set_camera(cam)


#   Optimization:
N_interactions = 500
learning_rate = 1e-6

# =======================================================
# =======================================================
# =======================================================

#   RUNNING:

    # Push before optimization
print("---Seed Simulation---")
push(0, steps, render)

    # Opt
print("\n---Optimization---")
force_x_opt = Optimization(target, N_interactions, steps, learning_rate)

    # Push after optimization
print("\n---Final Simulation---")
push(force_x_opt, steps, render)

sleep(2)


'''
while 1:
    for iter in range (50):
        if (iter == 49) :
            print(  "\n---Simulation---",
                    "\n | Mass:", mass_sphere,
                    "\n | Radius:", radius,
                    "\n | Target:", target[0],
                    "\n >>> Force_x:", force_x,
                    "\n >>> Loss:", loss)
'''