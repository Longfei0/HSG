#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" Module with auxiliary functions. """

import math
import numpy as np
import carla

def get_angle(target, ego_vector, ego_location, Symbols=False):
    '''
    para target : carla.Location
    para ego_location: carla.Location
    '''
    distance = distance_loc(target, ego_location)
    forward_vector = ego_vector
    actor_vector = carla.Vector3D(x=target.x - ego_location.x,
                                  y=target.y - ego_location.y, z=0)
    actor_vector = actor_vector / distance
    if is_left(forward_vector, actor_vector) and Symbols == True:
        x = forward_vector.x * actor_vector.x + forward_vector.y * actor_vector.y
        if abs(x) > 1:
            x = round(x)
        angle = -math.degrees(math.acos(x))
    else:
        x = forward_vector.x * actor_vector.x + forward_vector.y * actor_vector.y
        if abs(x) > 1:
            x = round(x)
        angle = math.degrees(math.acos(x))

    return angle


def get_angle_pure(vector1, vector2):

    vector1 = vector1 / (math.sqrt(vector1.x ** 2 + vector1.y ** 2))
    vector2 = vector2 / (math.sqrt(vector2.x ** 2 + vector2.y ** 2))

    x = vector1.x * vector2.x + vector1.y * vector2.y
    if abs(x) > 1:
        x = round(x)
    angle = math.degrees(math.acos(x))

    return angle


def fix_bounding_box(actor):
    """
        the bounding_box.location for every actor should be zero, so this value of bikes is wrong which have to be modified manually. Because of all
        the location in bounding box is zero, we need to change the location when we want to draw the bounding_box. For the environemnt objects, the
        location for each object just the location, not zero.
        actor: Vehicle_with_distance
    """
    if 'vehicle' in actor.v.type_id and actor.v.attributes["number_of_wheels"] == '4':
        actor.b = carla.BoundingBox(actor.v.get_transform().location + carla.Location(z=actor.v.bounding_box.extent.z), actor.v.bounding_box.extent)
    # elif 'vehicle' in actor.v.type_id and actor.v.attributes["number_of_wheels"] == '2' and actor.v.attributes["base_type"] == 'motorcycle':
    #     actor.b = carla.BoundingBox(actor.v.get_transform().location + carla.Location(z=actor.v.bounding_box.extent.z+0.35), actor.v.bounding_box.extent + carla.Vector3D(x=1.1, y=0.35, z=0.3))
    elif 'vehicle' in actor.v.type_id and actor.v.attributes["number_of_wheels"] == '2':
        actor.b = carla.BoundingBox(actor.v.get_transform().location + carla.Location(z=actor.v.bounding_box.extent.z+0.35), actor.v.bounding_box.extent + carla.Vector3D(x=0.85, y=0.35, z=0.3))
    elif 'walker' in actor.v.type_id:
        actor.b = carla.BoundingBox(actor.v.get_transform().location, actor.v.bounding_box.extent)


def is_left(forward_vector, distance_vector):
    """
        check if the vector is on the right/left side of forward_vector
        forward_vector: Carla.Vector3D
        distance_vector: Carla.Vector3D
    """
    # cross = a_x * b_y - a_y * b_x
    cross = distance_vector.x * forward_vector.y - distance_vector.y * forward_vector.x

    # determine positive or negative
    if cross > 0:
        return True  # a is on the left side of b
    elif cross < 0:
        return False  # a is on the right side of b
    else:
        return None  # a and b are collinear


def is_higher(ego_vehicle, blocked_vehicle, seen_vehicle, bias = 0.1):
    """
        This function is to detect whether the seen_vehicle blocked the blocked_vehicle in height.

        ego_vehicle: vehicle_with_distance
        blocked: vehicle_with_distance
        seen_vehicle: carla.Actor
    """
    blocked_h = blocked_vehicle.b.location.z + blocked_vehicle.b.extent.z
    seen_h = seen_vehicle.b.location.z + seen_vehicle.b.extent.z
    ego_h = ego_vehicle.location.z
    delta_h = seen_h - ego_h
    expected_h = delta_h*(blocked_vehicle.d/seen_vehicle.d)

    # when block vehicle's height is lower than expected_h, then it's fully blocked by height.
    if expected_h + bias >= (blocked_h - ego_h):
        return False
    else:
        return True


def get_vector_angle(vector, angle_degrees):
    """
        function: rotate the start_vector in an angle of angle_degrees input
        start_vector: carla.Vector3D
        angle_degrees: float
    """
    # Convert the angle to radians
    angle_radians = math.radians(angle_degrees)

    # Compute the sine and cosine of the angle
    cos_angle = math.cos(angle_radians)
    sin_angle = math.sin(angle_radians)

    # Compute the components of the rotated vector
    x = vector.x * cos_angle - vector.y * sin_angle
    y = vector.x * sin_angle + vector.y * cos_angle

    # Create the rotated vector
    rotated_vector = carla.Vector3D(x=x, y=y)

    return rotated_vector


def draw_waypoints(world, waypoints, z=0.5):
    """
    Draw a list of waypoints at a certain height given in z.

        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
    """
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=0.15)


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def get_acceleration(vehicle):
    """
    Compute acceleration of a vehicle in m/s2.

        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    acc = vehicle.get_acceleration()
    forward_vec = vehicle.get_transform().get_forward_vector()
    if get_angle_pure(acc, forward_vec) < 90:
        acc_symbol = math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)
    else:
        acc_symbol = -math.sqrt(acc.x ** 2 + acc.y ** 2 + acc.z ** 2)

    return acc_symbol

def get_throttle(vehicle):
    """
    Get throttle of a vehicle in [0, 1].

        :param vehicle: the vehicle for which speed is calculated
        :return: throttle
    """
    throttle = vehicle.get_control().throttle

    return throttle

def get_brake(vehicle):
    """
    Get Brake of a vehicle in [0, 1].

        :param vehicle: the vehicle for which speed is calculated
        :return: brake
    """
    brake = vehicle.get_control().brake

    return brake

def get_steer(vehicle):
    """
    Get steer of a vehicle in [-1, 1].

        :param vehicle: the vehicle for which speed is calculated
        :return: brake
    """
    steer = vehicle.get_control().steer

    return steer

def is_within_distance_ahead(target_transform, current_transform, max_distance):
    """
    Check if a target object is within a certain distance in front of a reference object.

    :param target_transform: location of the target object
    :param current_transform: location of the reference object
    :param orientation: orientation of the reference object
    :param max_distance: maximum allowed distance
    :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_transform.location.x - current_transform.location.x, target_transform.location.y - current_transform.location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    fwd = current_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle < 90.0

def is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up, d_angle_th_low=0):
    """
    Check if a target object is within a certain distance from a reference object.
    A vehicle in front would be something around 0 deg, while one behind around 180 deg.

        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :param max_distance: maximum allowed distance
        :param d_angle_th_up: upper thereshold for angle
        :param d_angle_th_low: low thereshold for angle (optional, default is 0)
        :return: True if target object is within max_distance ahead of the reference object
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    # If the vector is too short, we can simply stop here
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle_th_low < d_angle < d_angle_th_up


def compute_magnitude_angle(target_location, current_location, orientation):
    """
    Compute relative angle and distance between a target_location and a current_location

        :param target_location: location of the target object
        :param current_location: location of the reference object
        :param orientation: orientation of the reference object
        :return: a tuple composed by the distance to the object and the angle between both objects
    """
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    """
    Returns the 2D distance from a waypoint to a vehicle

        :param waypoint: actual waypoint
        :param vehicle_transform: transform of the target vehicle
    """
    loc = vehicle_transform.location
    x = waypoint.transform.location.x - loc.x
    y = waypoint.transform.location.y - loc.y

    return math.sqrt(x * x + y * y)

def distance_loc(target, ego_vehicle):
    """
    Returns the 2D distance from a waypoint to a vehicle

        :param target: carla.Location
        :param ego_vehicle: carla.Location
    """
    x = target.x - ego_vehicle.x
    y = target.y - ego_vehicle.y
    return math.sqrt(x * x + y * y)

def distance_min(target, ego_vehicle):
    '''
            para target : carla.actor
            para ego_vehicle: carla.Transform
        '''
    if 'vehicle' in target.type_id and target.attributes["number_of_wheels"] == '2':
        bbx = carla.BoundingBox(carla.Location(), target.bounding_box.extent + carla.Vector3D(x=1.1, y=0.35, z=0.3))
    else:
        bbx = target.bounding_box
    pts = bbx.get_world_vertices(target.get_transform())
    min_dis = np.inf
    for pt in pts:
        dis = distance_loc(pt, ego_vehicle.location)
        if dis < min_dis:
            min_dis = dis
    return min_dis


def get_angle(target, ego_vector, ego_location, Symbols=False):
    '''
    para target : carla.Location
    para ego_location: carla.Location
    '''
    distance = distance_loc(target, ego_location)
    forward_vector = ego_vector
    actor_vector = carla.Vector3D(x=target.x - ego_location.x,
                                  y=target.y - ego_location.y, z=0)
    actor_vector = actor_vector / distance
    if is_left(forward_vector, actor_vector) and Symbols == True:
        x = forward_vector.x * actor_vector.x + forward_vector.y * actor_vector.y
        if abs(x) > 1:
            x = round(x)
        angle = -math.degrees(math.acos(x))
    else:
        x = forward_vector.x * actor_vector.x + forward_vector.y * actor_vector.y
        if abs(x) > 1:
            x = round(x)
        angle = math.degrees(math.acos(x))

    return angle


def get_angle_min(target, ego_vehicle, Symbols=False, angle = 0):
    '''
        para target : carla.actor
        para ego_vehicle: carla.Transform
    '''
    if 'vehicle' in target.type_id and target.attributes["number_of_wheels"] == '2':
        bbx = carla.BoundingBox(carla.Location(), target.bounding_box.extent + carla.Vector3D(x=1.1, y=0.35, z=0.3))
    else:
        bbx = target.bounding_box
    pts = bbx.get_world_vertices(target.get_transform())
    min_judge = np.inf
    min_angle = np.inf
    forward_vector = ego_vehicle.get_forward_vector()
    judge_vector = get_vector_angle(forward_vector, angle)
    for pt in pts:
        judge = get_angle(pt, judge_vector, ego_vehicle.location, Symbols=True)
        if abs(judge) < min_judge:
            min_judge = judge
            if not Symbols:
                min_angle = get_angle(pt, ego_vehicle.get_forward_vector(), ego_vehicle.location)
            else:
                min_angle = get_angle(pt, ego_vehicle.get_forward_vector(), ego_vehicle.location, Symbols=True)
    return min_angle


def vector(location_1, location_2):
    """
    Returns the unit vector from location_1 to location_2

        :param location_1, location_2: carla.Location objects
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


def compute_distance(location_1, location_2):
    """
    Euclidean distance between 3D points

        :param location_1, location_2: 3D points
    """
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
    return norm


def positive(num):
    """
    Return the given number if positive, else 0

        :param num: value to check
    """
    return num if num > 0.0 else 0.0
