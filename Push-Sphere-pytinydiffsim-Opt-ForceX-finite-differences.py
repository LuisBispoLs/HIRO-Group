'''
Force optimization based on finite gradients.
>> Loss values too high;
>> Force does not converge for some simulations.
'''

from absl import app
from time import sleep
from random import uniform as rand
import pytinyopengl3 as gl
import pytinydiffsim as tds

def push(force_x, step, render):

    if render:
        app.renderer.remove_all_instances()

    bodies = []
    visuals = []

    #   BUILD WORLD:        ========================================
    world = tds.TinyWorld()
    world.gravity = tds.Vector3(0, 0, -9.81)
    #world.friction = tds.fraction(5, 10)
    #test = world.friction
    #print("\nFriction = ", test)

    force_components = tds.Vector3(force_x, 0, 0)


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
                #print("v=",v)
                b = bodies[v]
                pos = gl.TinyVector3f(tds.get_debug_double(b.world_pose.position[0]),
                                    tds.get_debug_double(b.world_pose.position[1]),
                                    tds.get_debug_double(b.world_pose.position[2]))
                #print("pos=",pos)
                orn = gl.TinyQuaternionf(0,0,0,1)
                app.renderer.write_single_instance_transform_to_cpu(pos, orn, visuals[v])
                
            app.renderer.update_camera(2)
            dg = gl.DrawGridData()
            #dg.drawAxis = True
            app.draw_grid(dg)

            #   INDICATING THE TARGET:      ========================================
            from_line = gl.TinyVector3f(target[0], target[1], target[2])
            to_line = gl.TinyVector3f(target[0], target[1], 4)
            color = gl.TinyVector3f(1,0,0)
            width = 3
            app.draw_text_3d("Target", target[0], target[1], 4, 1)
            app.renderer.draw_line(from_line,to_line,color,width)
            
            app.renderer.write_transforms()
            app.renderer.render_scene()
            app.swap_buffer()
    

    #   LOSS FUNCTION:      ========================================
    loss = (sphere_body.world_pose.position[0] - target[0])**2         # (final_sphere_pos_x - target_x)^2
    '''
    print("\nForce X = ", force_components[0])
    print("Sphere position = ", sphere_body.world_pose.position)
    print("Target = ", target)
    print("Loss = ", loss)
    '''
    #print("Sphere position = ", sphere_body.world_pose.position, "Target = ", target)

    return loss

def grad_finite(force_x, steps = 300, eps = tds.fraction(1,10000)):
    loss = push(force_x, steps, False)
    cx = push(force_x + eps, steps, False)
    d_force_x = (cx - loss) / eps

    return loss, d_force_x

# =======================================================
# =======================================================
# =============        SETUP:        ====================
render = True
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

dt = tds.fraction(1, 60)
'''
mass_sphere = 2.0
target = tds.Vector3(-5, 0, 0)
radius = tds.fraction(1, 2)
'''
init_force_x = 0
steps = 300

learning_rate = tds.fraction(100,1)#tds.fraction(200,1)
Max_N_interactions = 100
# =======================================================
# =======================================================
# =======================================================


while 1:
    target = tds.Vector3(rand(-8., 8.), 0, 0)
    mass_sphere = rand(0.1, 5.)
    radius = rand(0.2, 2.)

    # Push before optimization
    push(init_force_x, steps, render)

    force_x = init_force_x

    loss = 1
    i = 0

    # Gradient descent using gradients
    while (loss > 0.005):

        loss, d_force_x = grad_finite(force_x, steps)
        '''
        print(  "\nIteration #", i,
                "\n | Target:", target[0],
                "\n >>> Force_x:", force_x,
                "\n >>> Loss:", loss)
        '''
        if (i > Max_N_interactions): break

        # UPDATE THE FORCE:
        #   "The simplest update rule used in practice is the Stochastic Gradient Descent (SGD):"
        #   >>>     weight = weight - learning_rate * gradient
        force_x -= learning_rate * d_force_x
        i += 1


    if (i > Max_N_interactions):
        print(  "\n---Simulation Failed---",
                "\n | Mass:", mass_sphere,
                "\n | Radius:", radius)
        sleep(1)
    else:
        i_print = i-1
        print(  "\n---Simulation---",
                "\n | Mass:", mass_sphere,
                "\n | Radius:", radius,
                "\n | Target:", target[0],
                "\n |"
                "\n | Interactions: #", i_print,
                "\n | >>> Force_x:", force_x,
                "\n | >>> Loss:", loss)

        # Push after optimization
        push(force_x, steps, render)
        sleep(1)