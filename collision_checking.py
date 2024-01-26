import numpy as np
import fcl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


# Load data from the NPZ file
#loaded_data_agent1 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/N45W123/distributed/ak2/agent1/log.npz')
#loaded_data_agent2 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/N45W123/distributed/ak2/agent2/log.npz')

loaded_data_agent1 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/interpolated_data_normalized/distributed/ak2/agent1/log.npz')
loaded_data_agent2 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/interpolated_data_normalized/distributed/ak2/agent2/log.npz')

# Function to create a CollisionObject from x and y coordinates using spheres
def create_collision_sphere_object_from_trajectory(x, radius_factor=1):
    # Calculate the center and radius of the smallest bounding sphere around the trajectory
    center = np.array([x[0], x[1], 0], dtype=np.float64)
    #radius = max((x_coordinates.max() - x_coordinates.min()) / 2, (y_coordinates.max() - y_coordinates.min()) / 2) * radius_factor

    # Create a CollisionObject for the trajectory using a Sphere shape
    sphere = fcl.Sphere(radius_factor)
    transformation = fcl.Transform(center)
    collision_object = fcl.CollisionObject(sphere, transformation)
    #collision_object.setTranslation(center)

    return collision_object

def create_collision_box_object_from_trajectory(x, width, length, height=0):
    # Calculate the center and radius of the smallest bounding sphere around the trajectory
    #center = np.array([x[0], x[1], 0], dtype=np.float64)
    #radius = max((x_coordinates.max() - x_coordinates.min()) / 2, (y_coordinates.max() - y_coordinates.min()) / 2) * radius_factor

    center = np.array([x[0], x[1], 0], dtype=np.float64)
    # Create a CollisionObject for the trajectory using a Box shape
    box = fcl.Box(length, width, height)
    transformation = fcl.Transform(center)
    collision_object = fcl.CollisionObject(box, transformation)
    #collision_object.setTranslation(center)

    return collision_object


# Function to check for collisions pair-wise between two trajectories
def check_collision_pairwise(p1, p2):

    # Create Sphere CollisionObjects for each trajectory
    #collision_object1 = create_collision_sphere_object_from_trajectory(p1)
    #collision_object2 = create_collision_sphere_object_from_trajectory(p2)

    # Create Box CollisionObjects for each trajectory
    collision_object1 = create_collision_box_object_from_trajectory(p1, width=1, length=1)
    collision_object2 = create_collision_box_object_from_trajectory(p2, width=1, length=1)

    request = fcl.CollisionRequest()
    result = fcl.CollisionResult()
    
    fcl.collide(collision_object1, collision_object2, request, result)
    
    return result.is_collision

    
# Extract x and y coordinates from loaded data
xs_1 = loaded_data_agent1['xs']
xs_2 = loaded_data_agent2['xs']
ax = plt.gca()
plt.axis([-10, 10, -10, 10])
for p1,p2 in zip(xs_1, xs_2):
    #print(p1, p2)
    #o1 = create_collision_object_from_trajectory(p1)
    #o2 = create_collision_object_from_trajectory(p2)
    ax.scatter(p1[0],p1[1], c='blue')
    ax.scatter(p2[0],p2[1], c='red')
    # c1 = Circle(p1, radius=1, alpha=0.4, color="blue")
    # c2 = Circle(p2, radius=1, alpha=0.4, color="red")
    # ax.add_patch(c1)
    # ax.add_patch(c2)
    rec1 = Rectangle([p1[0]-0.5,p1[1]-0.75], width=1, height=1.5, alpha=0.4, color="blue")
    rec2 = Rectangle([p2[0]-0.5,p2[1]-0.75], width=1, height=1.5, alpha=0.4, color="red")
    ax.add_patch(rec1)
    ax.add_patch(rec2)
    if check_collision_pairwise(p1, p2):
        print("collision", p1, p2)
    plt.pause(0.1)
