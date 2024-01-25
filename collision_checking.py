import numpy as np
import fcl
import matplotlib.pyplot as plt
from fcl import CollisionObject, DynamicAABBTreeCollisionManager

# Load data from the NPZ file
#loaded_data_agent1 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/interpolated_data_normalized/distributed/ak2/agent1/log.npz')
#loaded_data_agent2 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/interpolated_data_normalized/distributed/ak2/agent2/log.npz')

loaded_data_agent1 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/N45W123/distributed/ak2/agent1/log.npz')
loaded_data_agent2 = np.load('/lclhome/plope113/LCD_RIG-master/outputs/2/N45W123/distributed/ak2/agent2/log.npz')


# Extract x and y coordinates from loaded data
xs_1 = loaded_data_agent1['xs']
xs_2 = loaded_data_agent2['xs']


# Extract x and y coordinates
x_coordinates1 = xs_1[:, 0]
y_coordinates1 = xs_1[:, 1]

x_coordinates2 = xs_2[:, 0]
y_coordinates2 = xs_2[:, 1]


# Function to create a CollisionObject from x and y coordinates
def create_collision_object_from_trajectory(x_coordinates, y_coordinates):
    # Create a single CollisionObject for the trajectory using a Box shape
    aabb_min = np.array([x_coordinates.min(), y_coordinates.min(), 0], dtype=np.float64)
    aabb_max = np.array([x_coordinates.max(), y_coordinates.max(), 0], dtype=np.float64)
    box = fcl.Box(aabb_max[0] - aabb_min[0], aabb_max[1] - aabb_min[1], 0.01)  # Adjust the dimensions as needed
    collision_object = CollisionObject(box, fcl.Transform())
    collision_object.setTranslation(aabb_min + (aabb_max - aabb_min) / 2)
    return collision_object

# Function to check for collisions between two trajectories
def check_collision_with_trajectory(x_coordinates1, y_coordinates1, x_coordinates2, y_coordinates2, agent1_id='Agent 1', agent2_id='Agent 2'):
    # Check if either trajectory is empty
    if x_coordinates1.size == 0 or x_coordinates2.size == 0:
        return False, None

    # Create CollisionObjects for each trajectory
    collision_object1 = create_collision_object_from_trajectory(x_coordinates1, y_coordinates1)
    collision_object2 = create_collision_object_from_trajectory(x_coordinates2, y_coordinates2)

    # Perform collision checking
    request = fcl.CollisionRequest()
    result = fcl.CollisionResult()
    collide = fcl.collide(collision_object1, collision_object2, request, result)

    if collide:
        print(f"{len(result.contacts)} collision(s) detected!")
        print("Contact Points:")
        collision_points = []
        for contact in result.contacts:
            x_position = contact.pos[0]
            y_position = contact.pos[1]
            collision_points.append(contact.pos)  # Append collision point to the list
            print(f"Collision at: ({x_position}, {y_position})")
        return True, collision_points  # Return the identifier of the trajectory that caused the collision
    else:
        print("No collision detected.")
        return False, None

# Check for collisions
collision_detected, colliding_points = check_collision_with_trajectory(x_coordinates1, y_coordinates1, x_coordinates2, y_coordinates2)
if collision_detected:
    print(f"Collision detected!")
else:
    print("No collision detected.")

# Plot trajectories
plt.figure(figsize=(8, 6))
# Plot robot 1 trajectory
plt.plot(x_coordinates1, y_coordinates1, label='Robot 1', color='blue')
plt.scatter(x_coordinates1[0], y_coordinates1[0], color='green', label='Robot 1 Start')
plt.scatter(x_coordinates1[-1], y_coordinates1[-1], color='red', label='Robot 1 End')

# Plot robot 2 trajectory
plt.plot(x_coordinates2, y_coordinates2, label='Robot 2', color='orange')
plt.scatter(x_coordinates2[0], y_coordinates2[0], color='limegreen', label='Robot 2 Start')
plt.scatter(x_coordinates2[-1], y_coordinates2[-1], color='maroon', label='Robot 2 End')

# Plot collision points
if colliding_points:
    collision_points = np.array(colliding_points)
    plt.scatter(collision_points[:, 0], collision_points[:, 1], color='red', label='Collision Points')

plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.title('Trajectories of Two Robots')
plt.legend()
plt.grid(True)
plt.show()


