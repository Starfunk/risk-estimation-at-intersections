"""Occlusion calculations"""
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from compute_line import compute_bresenham_line, compute_FVTA_line
import time

""" Constants """
pi = math.pi
car_length = 55
car_width = 26

# 5.5m = 55 decimeters = 550 cm
# 2.6m = 26 decimeter = 260 cm

fov_circle_points = 360
# Lower this number to increase efficiency,
fov_circle_points_scale = 2
# If fov_circle_points_scale = 2, then we have fov_circle_points = 720
fov_circle_points *= fov_circle_points_scale

occupancy_map_origin = (10,10)
occupancy_map_length = 180
occupancy_map_width = 180
occupancy_map_grid_size = 1

class Vehicle:
    def __init__(self, x, y, traffic_region,
                 velocity, acceleration,
                 angle, vehicle_type, id,
                 fov_distance, fov_range):
        self.x, self.y = x, y
        self.traffic_region = traffic_region
        self.velocity, self.acceleration = velocity, acceleration
        self.length, self.width = self.get_vehicle_size(vehicle_type)
        self.angle = angle
        self.id = id
        self.fov_distance = fov_distance
        self.fov_range = fov_range

    @staticmethod
    def get_vehicle_size(vehicle_type):
        if vehicle_type == "car":
            length = car_length
            width = car_width
            return length, width

    def get_bounding_box_points(self):
        # To make this more efficient, could get rid of division by 2 and
        # make length and width mean half length and width.
        # These define the four points for the vehicle's bounding box
        front_left = (self.x + self.length/2, self.y + self.width/2)
        front_right = (self.x + self.length/2, self.y - self.width/2)
        back_left = (self.x - self.length/2, self.y + self.width/2)
        back_right = (self.x - self.length/2, self.y - self.width/2)

        bounding_box_points = [front_left, front_right, back_left, back_right]
        rotated_bounding_box_points = []

        for point in bounding_box_points:
            rotated_x = (point[0] - self.x) * math.cos(self.angle) - \
                        (point[1] - self.y) * math.sin(self.angle) + self.x
            rotated_y = (point[0] - self.x) * math.sin(self.angle) + \
                        (point[1] - self.y) * math.cos(self.angle) + self.y
            rotated_bounding_box_points.append((int(round(rotated_x)),
                                                int(round(rotated_y))))
        return rotated_bounding_box_points


class OccupancyMap:
    """
        Occupancy Map:
               width →
        ---------------------
        |__| ← grid_size    |
        |                   |
        |                   | length ↑
        |                   |
        ---------------------
    """
    def __init__(self, origin=occupancy_map_origin,
                length=occupancy_map_length,
                 width=occupancy_map_width,
                 grid_size=occupancy_map_grid_size):
        self.origin = origin
        self.length = length
        self.width = width
        self.grid_size = grid_size
        self.grid = self.make_grid()

    # This may not be needed.
    def clear(self):
        self.grid = make_grid(length,width)

    """
        make_grid: Makes a "grid" in the form of a dictionary where the coordinates
        on the grid are the keys and each coordinate is associated with a list of
        vehicle ids that have occupied that grid square.
        grid = {
            (0,0): []
            .
            .
            (1,0): [1]
        }
    """
    def make_grid(self):
        grid = {}
        for y in range(self.origin[1], self.length + self.origin[1] + 1):
            for x in range(self.origin[0], self.width + self.origin[0] + 1):
                grid[(x,y)] = [[],[]]
        return grid

    """
    occupied: Returns true if the point, (px,py) lies within the grid square centred at
    (x,y) with side length, length.
    """
    def occupied(self,px,py,x,y):
        if (px <= x + self.grid_size / 2) and (px >= x - self.grid_size / 2) and \
           (py <= y + self.grid_size / 2) and (py >= y - self.grid_size / 2):
           return True
        else:
           return False

    # THIS NEEDS TO BE FIXED: POINTS OR VEHICLES???!!!
    # TODO: Could do some kind of binary search for the right area to map point?
    # This is really slow...searching across the entire grid for each point.
    def make_occupancy_map(self, vehicles):

        for vehicle in vehicles:
            bounding_box_points = vehicle.get_bounding_box_points()

            l1 = compute_FVTA_line(bounding_box_points[0][0],
                                   bounding_box_points[0][1],
                                   bounding_box_points[1][0],
                                   bounding_box_points[1][1])

            l2 = compute_FVTA_line(bounding_box_points[0][0],
                                   bounding_box_points[0][1],
                                   bounding_box_points[2][0],
                                   bounding_box_points[2][1])

            l3 = compute_FVTA_line(bounding_box_points[1][0],
                                   bounding_box_points[1][1],
                                   bounding_box_points[3][0],
                                   bounding_box_points[3][1])

            l4 = compute_FVTA_line(bounding_box_points[2][0],
                                   bounding_box_points[2][1],
                                   bounding_box_points[3][0],
                                   bounding_box_points[3][1])

            bounding_box = l1 + l2 + l3 + l4

            for point in bounding_box:

                # Make sure the bounding box point is within the bounds of the map
                if point[0] > occupancy_map_width + self.origin[0] or \
                   point[1] > occupancy_map_length + self.origin[1]:
                    continue
                if point[0] < self.origin[0] or point[1] < self.origin[1]:
                    continue

                self.grid[(int(round(point[0])),
                           int(round(point[1])))][0].append(vehicle.id)
        return

    """
    compute_occlusion: get how many raycasts intersect with the vehicle with
    the input id.
    This is an example of a grid in the occupancy map. First we have the coordinates
    of the grid point. Next we have the vehicles occupying the point (the first list)
    Next, we have the vehicles that can see the point (the second list)
    grid squre = ((78, 98), [[1], [0, 1]])
    """

    # This is only to compute occlusion subject and target vehicles.
    def compute_occlusion(self, raycasts, subject_vehicle_id, target_vehicle_id):
        hit_count = 0
        visited_grid_squares = []

        for raycast in raycasts:
            # Make sure points all go from subject vehicle to target vehicle
            for point in raycast:

                # Outside grid bounds
                if point[0] > occupancy_map_width + self.origin[0] or \
                   point[1] > occupancy_map_length + self.origin[1]:
                    break
                if point[0] < self.origin[0] or point[1] < self.origin[1]:
                    break

                if point in visited_grid_squares and target_vehicle_id in self.grid[point][0]:
                    break

                # Case where the subject vehicle can see a car that is not the target vehicle
                if len(self.grid[point][0]) != 0 and target_vehicle_id not in self.grid[point][0]:
                    # Make sure the subject vehicle is not just seeing itself.
                    if len(self.grid[point][0]) == 1 and subject_vehicle_id in self.grid[point][0]:
                        continue
                    else:
                        self.grid[point][1].append(subject_vehicle_id)
                        break

                if target_vehicle_id in self.grid[point][0] and point not in visited_grid_squares:
                    visited_grid_squares.append(point)
                    self.grid[point][1].append(subject_vehicle_id)
                    hit_count += 1
                    break


        return hit_count



    def visualize_occupancy_map(self, vehicle1_id, vehicle2_id):
        # Create figure and axes
        fig, ax = plt.subplots()

        for grid_square in self.grid.items():

            # e.g., grid squre = ((78, 98), [[0],[1]])
            # This means at this point, x = 78, y = 98, the vehicle with id
            # 0 occupies the grid square and vehicle 1 can see this grid square.

            # other parameters:  edgecolor='r'
            # Draw purple for intersection between two fov regions
            if vehicle1_id in grid_square[1][1] and vehicle2_id in grid_square[1][1]: # DO SAME THING WITH OTHER LINES
                plotted_grid_square = patches.Rectangle((grid_square[0][0], grid_square[0][1]), \
                    width=self.grid_size, height=self.grid_size, linewidth=1, facecolor='purple')
                ax.add_patch(plotted_grid_square)

            # Draw blue for fov region for vehicle 0
            # This is
            elif vehicle1_id in grid_square[1][1]:
                plotted_grid_square = patches.Rectangle((grid_square[0][0], grid_square[0][1]), \
                    width=self.grid_size, height=self.grid_size, linewidth=1, facecolor='blue')
                ax.add_patch(plotted_grid_square)

            # Draw red for fov region for vehicle 1
            elif vehicle2_id in grid_square[1][1]:
                plotted_grid_square = patches.Rectangle((grid_square[0][0], grid_square[0][1]), \
                    width=self.grid_size, height=self.grid_size, linewidth=1, facecolor='red')
                ax.add_patch(plotted_grid_square)

            # Draw black to signal this is bounding box of a vehicle.
            elif grid_square[1][0]:
                    plotted_grid_square = patches.Rectangle((grid_square[0][0], grid_square[0][1]), \
                        width=self.grid_size, height=self.grid_size, linewidth=1, facecolor='lightgrey')
                    ax.add_patch(plotted_grid_square)

        plt.xlim([self.origin[0], self.width + self.origin[0] + 1])
        plt.ylim([self.origin[1], self.length + self.origin[1] + 1])
        plt.show()


    def add_raycast(self, raycasts, subject_vehicle_id):
        for raycast in raycasts:
            for point in raycast:
                if point[0] > occupancy_map_width + self.origin[0] or \
                   point[1] > occupancy_map_length + self.origin[1]:
                    break
                if point[0] < self.origin[0] or point[1] < self.origin[1]:
                    break
                self.grid[point][1].append(subject_vehicle_id)
                # grid squre = ((78, 98), [[2], [0]])
                if subject_vehicle_id in self.grid[point][1] \
                    and len(self.grid[point][0]) > 0 and \
                    subject_vehicle_id not in self.grid[point][0]:
                    break


def compute_angle(vehicle1,vehicle2):
    x = vehicle2.x - vehicle1.x
    y = vehicle2.y - vehicle1.y
    angle = math.atan2(y,x)
    if angle < 0:
        angle = angle + 2 * pi
    return angle


"""
    make_circle: Outputs points on a circle with the given radius centred
    at (x_offset, y_offset). The number n determines how many equally
    spaced points will be created on the circle.
"""
def make_circle(radius,x_offset,y_offset,n=fov_circle_points):
    return [(math.cos(2*pi/n*i)*radius + x_offset, math.sin(2*pi/n*i)*radius + y_offset) for i in range(0,n+1)]


def angle_check(angle):
    if angle > 2 * pi:
        return angle - 2 * pi
    elif angle < 0:
        return angle + 2 * pi
    else:
        return angle

def occlusion_check(vehicle1,vehicle2,occupancy_map):

    # Calculate the angle between both vehicles
    v1_v2_angle = compute_angle(vehicle1,vehicle2)
    v2_v1_angle = compute_angle(vehicle2,vehicle1)

    v1_fov = make_circle(vehicle1.fov_distance, vehicle1.x, vehicle1.y)

    v1_min_angle = int(round(angle_check(v1_v2_angle - vehicle1.fov_range) * 180 / pi)) * fov_circle_points_scale
    v1_max_angle = int(round(angle_check(v1_v2_angle + vehicle1.fov_range) * 180 / pi)) * fov_circle_points_scale

    if v1_min_angle > v1_max_angle:
        v1_fov = v1_fov[v1_max_angle:v1_min_angle]
    else:
        v1_fov = v1_fov[v1_min_angle:v1_max_angle]

    v2_fov = make_circle(vehicle2.fov_distance, vehicle2.x, vehicle2.y)
    v2_min_angle = int(round(angle_check(v2_v1_angle - vehicle2.fov_range) * 180 / pi)) * fov_circle_points_scale
    v2_max_angle = int(round(angle_check(v2_v1_angle + vehicle2.fov_range) * 180 / pi)) * fov_circle_points_scale

    if v2_min_angle > v2_max_angle:
        v2_fov = v2_fov[v2_max_angle:v2_min_angle]
    else:
        v2_fov = v2_fov[v2_min_angle:v2_max_angle]

    # Constructing 2D raycast for vehicle1:
    vehicle1_x = int(round(vehicle1.x))
    vehicle1_y = int(round(vehicle1.y))
    vehicle1_raycasts = []
    for point in v1_fov:
        fov_x = int(round(point[0]))
        fov_y = int(round(point[1]))
        raycast = compute_FVTA_line(vehicle1_x, vehicle1_y, fov_x, fov_y)
        vehicle1_raycasts.append(raycast)

    # Constructing 2D raycast for vehicle2:
    vehicle2_x = int(round(vehicle2.x))
    vehicle2_y = int(round(vehicle2.y))
    vehicle2_raycasts = []
    for point in v2_fov:
        fov_x = int(round(point[0]))
        fov_y = int(round(point[1]))
        raycast = compute_FVTA_line(vehicle2_x, vehicle2_y, fov_x, fov_y)
        vehicle2_raycasts.append(raycast)

    # Computing unoccluded hit points i.e., just vehicle 1 and vehicle 2
    # are on the road.
    vehicles = [vehicle1,vehicle2]
    unoccluded_occupancy_map = OccupancyMap()
    unoccluded_occupancy_map.make_occupancy_map(vehicles)

    unoccluded_v2_hit_count = unoccluded_occupancy_map.compute_occlusion(vehicle1_raycasts, vehicle1.id, vehicle2.id)

    unoccluded_v1_hit_count = unoccluded_occupancy_map.compute_occlusion(vehicle2_raycasts, vehicle2.id, vehicle1.id)

    # COMPUTING WHICH RAYS FROM VEHICLE1 HIT VEHICLE2
    occluded_v2_hit_count = occupancy_map.compute_occlusion(vehicle1_raycasts, vehicle1.id, vehicle2.id)

    # COMPUTING WHICH RAYS FROM VEHICLE2 HIT VEHICLE1
    occluded_v1_hit_count = occupancy_map.compute_occlusion(vehicle2_raycasts, vehicle2.id, vehicle1.id)

    # TO VISUALIZE RAYCASTS UNCOMMENT THESE
    # FOR VEHICLE1
    # occluded_occupancy_map.add_raycast(vehicle1_raycasts, vehicle1.id)
    # FOR VEHICLE2
    # occluded_occupancy_map.add_raycast(vehicle2_raycasts, vehicle2.id)

    # TO VISUALIZE OCCUPANCY MAP UNCOMMENT THIS
    # occluded_occupancy_map.visualize_occupancy_map(vehicle1.id,vehicle2.id)


    v2_occlusion_rate = (unoccluded_v2_hit_count - occluded_v2_hit_count) / unoccluded_v2_hit_count
    v1_occlusion_rate = (unoccluded_v1_hit_count - occluded_v1_hit_count) / unoccluded_v1_hit_count

    return v2_occlusion_rate, v1_occlusion_rate


"""
This defines a car.
"""
if __name__ == '__main__':

    x, y = 40, 60
    traffic_region = "l_n"
    velocity, acceleration = 2, 0.1
    angle = pi * 0.73
    vehicle_type = "car"
    id = 0
    fov_distance = 150
    fov_range = pi / 6 # Max number is pi /2
    v0 = Vehicle(x, y, traffic_region, velocity, acceleration, angle, vehicle_type, id, fov_distance, fov_range)


    x, y = 80, 90
    traffic_region = "l_n"
    velocity, acceleration = 2, 0.1
    angle = pi * 2.3
    vehicle_type = "car"
    id = 1
    fov_distance = 150 # measured in 10 cm chunks?
    fov_range = pi / 6
    v1 = Vehicle(x, y, traffic_region, velocity, acceleration, angle, vehicle_type, id, fov_distance, fov_range)


    x, y = 120, 150
    traffic_region = "l_n"
    velocity, acceleration = 2, 0.1
    angle = pi * 2.8
    vehicle_type = "car"
    id = 2
    fov_distance = 150 # measured in 10 cm chunks?
    fov_range = pi / 6
    v2 = Vehicle(x, y, traffic_region, velocity, acceleration, angle, vehicle_type, id, fov_distance, fov_range)



    vehicles = [v0,v1,v2]
    occluded_occupancy_map = OccupancyMap()
    occluded_occupancy_map.make_occupancy_map(vehicles)

    v2_occlusion_rate, v1_occlusion_rate = occlusion_check(v0,v2, occluded_occupancy_map)
    print(v2_occlusion_rate)
    print(v1_occlusion_rate)
