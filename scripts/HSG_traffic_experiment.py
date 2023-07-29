#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import glob
import os
import sys
import time
import matplotlib.pyplot as plt
import numpy as np

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
from carla import VehicleLightState as vls

import argparse
import logging
from tools.misc_plus import get_speed, get_acceleration, get_throttle, get_brake, fix_bounding_box


class agent_counter:
    """
        A class that store the Hazard3 offset counts and can be used for update every click
    """

    def __init__(self, actor, offset_count=0, dis_count=0, speed_count=0):
        self.next_offset_count = offset_count
        self.next_dis_count = dis_count
        self.next_speed_count = speed_count
        self.vehicle = actor
        self.controloss_start = []  # here to save the counts when the control loss starts
        self.controloss_end = []  # here to save the counts when the control loss ends

    def get_ControlLoss_interval(self, experiment_t, delta_seconds, t_loss, interval=30):
        """
            Input:
                experiment_t: (int) the total experiment time in minutes
                delta_seconds: (float) usually the carla.setting.fixed_delta_seconds
                interval: (int) the interval when each time the vehicle will control loss once

            output:
                control_start: (list) a list of all the control loss start time in total experiment time
                control_end: (list) a list of all the control loss end time in total experiment time
        """

        indexs = int(experiment_t * 60 / interval)
        for i in range(indexs):
            t_last = np.clip(np.random.normal(1.5, 0.5), t_loss[0], t_loss[1])
            t_start = np.random.uniform(0 + t_last, interval - t_last)
            start = int(
                t_start / delta_seconds) + i * interval / delta_seconds
            end = start + int(t_last / delta_seconds)
            self.controloss_start.append(start)
            self.controloss_end.append(end)

        # this is to make sure the list won't out of range
        self.controloss_start.extend([0, 0])
        self.controloss_end.extend([0, 0])


# Create a set to store unique collision pairs
unique_collisions = set()

# Callback function for collision events
def collision_callback(event, Collision_Counter, world, blueprints, spawn_points, collision_bp, traffic_manager, hazard3_agents, hazard4_agents, hazard5_agents):
    """
        Callback function to handle collision events.
    """
    global unique_collisions
    # Get the vehicle ids involved in the collision
    vehicle1 = event.actor
    vehicle2 = event.other_actor
    vehicle1_id = vehicle1.id
    vehicle2_id = vehicle2.id

    # Create a unique collision pair (sorted tuple to avoid duplicates)
    collision_pair = tuple(sorted((vehicle1_id, vehicle2_id)))
    if collision_pair not in unique_collisions:
        # Increment the hazard count based on the group the vehicles belong to
        for idx in collision_pair:
            if idx == vehicle1_id:
                actor = vehicle1
            else:
                actor = vehicle2

            if 'role_name' in actor.attributes:
                for i in range(5):
                    if ('hazard' + str(i + 1)) in actor.attributes['role_name']:
                        Collision_Counter[i] += 1
                    elif 'normal' in actor.attributes['role_name']:
                        Collision_Counter[5] += 1
                        break
        # Add the collision pair to the set
        unique_collisions.add(collision_pair)

        # here to respawn the actor that being deleted
        actor_list = []
        bp_list = []
        v_list = []
        vehicle_list = [vehicle1, vehicle2]
        for i, actor in enumerate(vehicle_list):
            bp = np.random.choice(blueprints)
            if 'role_name' in actor.attributes:
                bp.set_attribute('role_name', actor.attributes['role_name'])
                bp_list.append(bp)
                v_list.append(actor)

        # Destroy the vehicles involved in the collision
        print(f'destroy {vehicle1.type_id} and {vehicle2.type_id}')

        for actor in v_list:
            actor.destroy()

        for bp in bp_list:
            for pt in spawn_points:
                actor = world.try_spawn_actor(bp, pt)
                if actor:
                    actor.set_autopilot(True, traffic_manager.get_port())
                    actor_list.append(actor)
                    collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))
                    break

        for i, actor in enumerate(actor_list):
            if 'hazard1' in actor.attributes['role_name']:
                # Use TM to set the behaviors of hazard1 agents
                traffic_manager.vehicle_percentage_speed_difference(actor, -50)  # over speed 40%
                traffic_manager.distance_to_leading_vehicle(actor, 1.5)
                print('respawn a hazard1 vehicle')
            elif 'hazard2' in actor.attributes['role_name']:
                # Use TM to set the behaviors of hazard2 agents
                traffic_manager.vehicle_percentage_speed_difference(actor, 50)  # over speed 40%
                traffic_manager.distance_to_leading_vehicle(actor, 4.5)
                print('respawn a hazard2 vehicle')
            elif 'hazard3' in actor.attributes['role_name']:
                # Use TM to set the behaviors of hazard3 agents
                for agent in hazard3_agents:
                    if agent.vehicle.id == v_list[i]:
                        agent.vehicle = actor
                print('respawn a hazard3 vehicle')
            elif 'hazard4' in actor.attributes['role_name']:
                # Use TM to set the behaviors of hazard4 agents
                traffic_manager.ignore_lights_percentage(actor, 50)
                traffic_manager.ignore_signs_percentage(actor, 50)
                for agent in hazard4_agents:
                    if agent.vehicle.id == v_list[i]:
                        agent.vehicle = actor
                print('respawn a hazard4 vehicle')
            elif 'hazard5' in actor.attributes['role_name']:
                for agent in hazard5_agents:
                    if agent.vehicle.id == v_list[i]:
                        agent.vehicle = actor
                print('respawn a hazard5 vehicle')
            else:
                traffic_manager.vehicle_percentage_speed_difference(actor, 0)  # over speed 40%
                traffic_manager.distance_to_leading_vehicle(actor, 3)
                print("respawn a normal vehicle")

    # print(collided_actor.type_id)
    # print(f'frame: {event.frame}; timestamp: {event.timestamp};  {hazard_actor.attributes["role_name"]} hit the {collided_actor.attributes["role_name"]} the overall count is {Collision_Counter}')


def collision_callback_respawn(event, world, blueprints, collision_bp, spawn_points, sensor_list, Collision_Counter):
    """
        Callback function to handle collision events.
        input: respawn_data(list): A list contains:
    """

    hazard_actor = event.actor
    collided_actor = event.other_actor
    # print(f'frame: {event.frame}; timestamp: {event.timestamp};  {actor_be_collided.type_id} Collision detected with: {collided_actor.type_id};  normal_impulse: {event.normal_impulse}')
    for i in range(5):
        if ('hazard' + str(i + 1)) in hazard_actor.attributes:
            Collision_Counter[i] += 1

    blueprint = np.random.choice(blueprints)
    blueprint.set_attribute('role_name', hazard_actor.attributes['role_name'])
    for pt in spawn_points:
        actor = world.try_spawn_actor(blueprint, pt)
        if actor:
            collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))
            break

    hazard_actor.destroy()


def bimodal_sample(mu1, sigma1, mu2, sigma2, p1=0.5, size=1):
    """
    Generate random samples from a bimodal distribution.

    Args:
        mu1: Mean of the first normal distribution.
        sigma1: Standard deviation of the first normal distribution.
        mu2: Mean of the second normal distribution.
        sigma2: Standard deviation of the second normal distribution.
        p1: Probability of choosing a sample from the first distribution.
        size: Number of samples to generate.

    Returns:
        A NumPy array of samples from the bimodal distribution.
    """
    # Generate an array of random probabilities to determine which distribution to sample from
    random_probs = np.random.rand(size)

    # Initialize an empty array to store the samples
    samples = np.empty(size)

    # Generate samples from the two normal distributions
    samples1 = np.random.normal(mu1, sigma1, size)
    samples2 = np.random.normal(mu2, sigma2, size)

    # Choose samples from the first distribution based on the probability p1
    samples[random_probs <= p1] = samples1[random_probs <= p1]

    # Choose samples from the second distribution based on the probability (1 - p1)
    samples[random_probs > p1] = samples2[random_probs > p1]

    return samples


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def draw_graphs():
    pass


def main():
    global collision_sensor_list, collision_counter
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-m', '--map',
        help='load a new map, use --list to see available maps')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=50,
        type=int,
        help='Number of vehicles (default: 30)')
    argparser.add_argument(
        '-nh', '--number-of-hazard-vehicles',
        metavar='N',
        default=1,
        type=int,
        help='--number-of-5-kinds-hazard-vehicles (default: 1)')
    argparser.add_argument(
        '-w', '--number-of-walkers',
        metavar='W',
        default=0,
        type=int,
        help='Number of walkers (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        default=True,
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--hybrid',
        action='store_true',
        help='Activate hybrid mode for Traffic Manager')
    argparser.add_argument(
        '-s', '--seed',
        metavar='S',
        default=9,
        type=int,
        help='Set random device seed and deterministic mode for Traffic Manager')
    argparser.add_argument(
        '--seedw',
        metavar='S',
        default=0,
        type=int,
        help='Set the seed for pedestrians module')
    argparser.add_argument(
        '--car-lights-on',
        action='store_true',
        default=False,
        help='Enable automatic car light management')
    argparser.add_argument(
        '--hero',
        action='store_true',
        default=False,
        help='Set one of the vehicles as hero')
    argparser.add_argument(
        '--respawn',
        action='store_true',
        default=False,
        help='Automatically respawn dormant vehicles (only in large maps)')
    argparser.add_argument(
        '--no-rendering',
        action='store_true',
        default=False,
        help='Activate no rendering mode')
    argparser.add_argument(
        '--debug',
        action='store_true',
        default=False,
        help='Keep the spector upon the hero vehicle for debug')
    argparser.add_argument(
        '--opt',
        action='store_true',
        default=False,
        help='Keep the spector upon the hero vehicle for debug')
    argparser.add_argument(
        '--duration', '-t',
        metavar='S',
        default=10,
        type=int,
        help='duration of the experiment (minutes)')
    argparser.add_argument(
        '--R-s',
        metavar='S',
        default=1.5,
        type=int,
        help='ratio of speeding to the limited speed 30 km/h. Default R_speed = 1.5')
    argparser.add_argument(
        '--R-i',
        metavar='S',
        default=0.5,
        type=int,
        help='ratio of impeding to the limited speed 30 km/h. Default R_imped = 0.5')
    argparser.add_argument(
        '--D-s',
        metavar='S',
        default=1.5,
        type=int,
        help='The distance the speeding vehicle will keep from the car in front')
    argparser.add_argument(
        '--D-i',
        metavar='S',
        default=6,
        type=int,
        help='The distance the impeding vehicle will keep from the car in front')
    argparser.add_argument(
        '--d-offset',
        metavar='S',
        default=1.7,
        type=int,
        help='the maximum distance offset of crimping vehicle')
    argparser.add_argument(
        '--t-next',
        metavar='S',
        default=[2, 10],
        type=list,
        help='Next state (speed ratio and following distance) reset time')
    argparser.add_argument(
        '--R-red',
        metavar='S',
        default=0.5,
        type=int,
        help='Percentage of hero vehicle will running the red light')
    argparser.add_argument(
        '--t-loss',
        metavar='S',
        default=[1, 3],
        type=list,
        help='the time range will each loss of vehicle control last')
    argparser.add_argument(
        '--t-interval',
        metavar='S',
        default=30,
        type=int,
        help='How long does the vehicle loss control at one time')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed)
    np.random.seed(args.seed)

    # remove all the static vehicles first
    # print(client.get_available_maps())
    if args.map is not None and args.opt is not None:
        print('load map %r.' % args.map)
        world = client.load_world(args.map, map_layers=carla.MapLayer.Ground)
        world.load_map_layer(carla.MapLayer.Buildings)
    elif args.map is not None:
        print('load map %r.' % args.map)
        world = client.load_world(args.map)

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        # here to set the safety parameter
        traffic_manager.set_global_distance_to_leading_vehicle(2)
        if args.respawn:
            traffic_manager.set_respawn_dormant_vehicles(True)
        if args.hybrid:
            traffic_manager.set_hybrid_physics_mode(True)
            traffic_manager.set_hybrid_physics_radius(70.0)
        if args.seed is not None:
            traffic_manager.set_random_device_seed(args.seed)

        settings = world.get_settings()
        if not args.asynch:
            traffic_manager.set_synchronous_mode(True)
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        if args.no_rendering:
            settings.no_rendering_mode = True
        world.apply_settings(settings)

        # TODO filter all the bikes
        collision_bp = world.get_blueprint_library().find('sensor.other.collision')
        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)
        blueprintsWalkers = get_actor_blueprints(world, args.filterw, args.generationw)

        if args.safe:
            blueprints = [x for x in blueprints if x.get_attribute('base_type') == 'car']

        blueprints = sorted(blueprints, key=lambda bp: bp.id)

        carla_map = world.get_map()
        spawn_points = carla_map.get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        # todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------

        # decide how much vehicle will be generated for each type of hazard vehicle
        each_number = args.number_of_hazard_vehicles
        print(f'generate {each_number} each kinds of hazard vehicles')
        # Distribute the 5N sequence numbers equally into 5 different lists
        idx = random.sample(range(args.number_of_vehicles), 5 * each_number)
        hazard1_i, hazard2_i, hazard3_i, hazard4_i, hazard5_i = [idx[i::5] for i in range(5)]

        batch = []
        random.shuffle(spawn_points)
        for i, transform in enumerate(spawn_points):
            if i >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if i in hazard1_i:
                blueprint.set_attribute('role_name', f'vehicle{i}_' + 'hazard1')
            elif i in hazard2_i:
                blueprint.set_attribute('role_name', f'vehicle{i}_' + 'hazard2')
            elif i in hazard3_i:
                blueprint.set_attribute('role_name', f'vehicle{i}_' + 'hazard3')
            elif i in hazard4_i:
                blueprint.set_attribute('role_name', f'vehicle{i}_' + 'hazard4')
            elif i in hazard5_i:
                blueprint.set_attribute('role_name', f'vehicle{i}_' + 'hazard5')
            else:
                blueprint.set_attribute('role_name', f'vehicle{i}_' + 'normal')
            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                         .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # -----------------------------------------------------------------------------------------------------------------------
        # HSG realization below
        # -----------------------------------------------------------------------------------------------------------------------

        # ---------------
        # Global Settings
        # ---------------

        traffic_manager.global_percentage_speed_difference(0)
        traffic_manager.set_global_distance_to_leading_vehicle(3)
        # traffic_manager.auto_lane_change(actor, True)
        # traffic_manager.collision_detection(reference_actor, other_actor, bool)
        # traffic_manager.distance_to_leading_vehicle(actor, distance)
        # traffic_manager.ignore_lights_percentage(actor, perc)
        # traffic_manager.ignore_signs_percentage(actor, perc)
        # traffic_manager.ignore_vehicles_percentage(actor, perc)
        # traffic_manager.vehicle_lane_offset(actor, offset)
        # traffic_manager.vehicle_percentage_speed_difference(actor, -40)
        # traffic_manager.get_next_action(actor)
        # traffic_manager.set_global_distance_to_leading_vehicle(distance)
        # traffic_manager.set_path(actor, path)

        # ---------------
        # initializaion
        # ---------------

        experiment_time = args.duration  # in minutes( int )
        print(f'this simulation will last for {experiment_time} minutes in simulator')
        ControlLoss_interval = args.t_interval  # in seconds (int)
        hazard3_agents = []
        hazard4_agents = []
        hazard5_agents = []
        collision_sensor_list = []

        # counting flags
        global_count = 0

        # vehicle categories
        list_names = ['normal', 'hazard1', 'hazard2', 'hazard3', 'hazard4', 'hazard5']
        lists = {name: [] for name in list_names}

        all_vehicle_actors = world.get_actors(vehicles_list)
        for i, actor in enumerate(all_vehicle_actors):
            if i in hazard1_i:
                lists['hazard1'].append(actor)
                collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))

                # Use TM to set the behaviors of hazard1 agents
                traffic_manager.vehicle_percentage_speed_difference(actor, 100 * (1 - args.R_s))  # over speed 50%
                traffic_manager.distance_to_leading_vehicle(actor, args.D_s)
            elif i in hazard2_i:
                lists['hazard2'].append(actor)
                collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))

                # Use TM to set the behaviors of hazard2 agents
                traffic_manager.vehicle_percentage_speed_difference(actor, 100 * (1 - args.R_i))  # less than speed 50%
                traffic_manager.distance_to_leading_vehicle(actor, args.D_i)
            elif i in hazard3_i:
                lists['hazard3'].append(actor)
                collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))

                # Use TM to set the behaviors of hazard3 agents
                hazard3_agents.append(agent_counter(actor))
            elif i in hazard4_i:
                lists['hazard4'].append(actor)
                collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))

                # Use TM to set the behaviors of hazard4 agents
                traffic_manager.ignore_lights_percentage(actor, 100 * args.R_red)
                traffic_manager.ignore_signs_percentage(actor, 100 * args.R_red)
                hazard4_agents.append(agent_counter(actor))
            elif i in hazard5_i:
                lists['hazard5'].append(actor)
                collision_sensor_list.append(world.spawn_actor(collision_bp, carla.Transform(), attach_to=actor))

                agent = agent_counter(actor)
                agent.get_ControlLoss_interval(experiment_time, settings.fixed_delta_seconds, args.t_loss, ControlLoss_interval)
                hazard5_agents.append(agent)
            else:
                lists['normal'].append(actor)

        # collision counter for: [hazard1, hazard2, hazard3, hazard4, hazard5]
        collision_counter = [0, 0, 0, 0, 0, 0]
        # make all sensor start working
        for n, sensor in enumerate(collision_sensor_list):
            sensor.listen(lambda event: collision_callback(event, collision_counter, world, blueprints, spawn_points, collision_bp, traffic_manager, hazard3_agents, hazard4_agents, hazard5_agents))

        while True:
            if not args.asynch and synchronous_master:

                for agent in hazard3_agents:
                    if global_count == agent.next_offset_count:
                        offset = np.clip(bimodal_sample(mu1=-args.d_offset/2, sigma1=0.5, mu2=args.d_offset/2, sigma2=0.5)[0], -args.d_offset, args.d_offset)
                        traffic_manager.vehicle_lane_offset(agent.vehicle, offset)
                        # set the offset distance reset time for the next time
                        agent.next_offset_count = global_count + int(
                            random.uniform(args.t_next[0], args.t_next[1]) / settings.fixed_delta_seconds)

                for agent in hazard4_agents:
                    if global_count == agent.next_dis_count:
                        dis = np.clip(np.random.normal(2.5, 0.75), args.D_s, args.D_i)
                        traffic_manager.distance_to_leading_vehicle(agent.vehicle, dis)
                        # set the offset distance reset time for the next time
                        agent.next_dis_count = global_count + int(
                            random.uniform(args.t_next[0], args.t_next[1]) / settings.fixed_delta_seconds)

                    if global_count == agent.next_speed_count:
                        speed = np.clip(np.random.normal(0, 25), 100 * (1 - args.R_s), 100 * (1 - args.D_i))
                        traffic_manager.vehicle_percentage_speed_difference(agent.vehicle, speed)
                        # set the offset distance reset time for the next time
                        agent.next_speed_count = global_count + int(
                            random.uniform(args.t_next[0], args.t_next[1]) / settings.fixed_delta_seconds)

                for agent in hazard5_agents:
                    if agent.controloss_start[0] == global_count:
                        agent.vehicle.set_autopilot(False, traffic_manager.get_port())
                        control = agent.vehicle.get_control()
                        t = round((agent.controloss_end[0] - agent.controloss_start[0]) * settings.fixed_delta_seconds,
                                  2)
                        # print(f'I am gonna lose control for {t} seconds')
                    elif agent.controloss_start[0] < global_count < agent.controloss_end[0]:
                        agent.vehicle.apply_control(control)

                        # draw the box of control loss vehicle
                        fix_box = carla.BoundingBox(agent.vehicle.get_transform().location + carla.Location(
                            z=agent.vehicle.bounding_box.extent.z), agent.vehicle.bounding_box.extent)
                        world.debug.draw_box(fix_box, agent.vehicle.get_transform().rotation, 0.05,
                                             carla.Color(100, 100, 0), 0.15)

                    elif agent.controloss_end[0] == global_count:
                        agent.controloss_start.pop(0)
                        agent.controloss_end.pop(0)
                        # print('control recovered')
                        agent.vehicle.set_autopilot(True, traffic_manager.get_port())

                if not args.debug:
                    # top view: only watch offset vehicle here
                    spectator = world.get_spectator()
                    view_time = 10  # how much time we are watching for each vehicle (in second)
                    count_interval = view_time / settings.fixed_delta_seconds
                    view_list_v = lists['hazard1'] + lists['hazard2'] + lists['hazard3'] + lists['hazard4']
                    vehicle_index = int(global_count / count_interval) % len(view_list_v)

                    transform = view_list_v[vehicle_index].get_transform()
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                            carla.Rotation(pitch=-90, yaw=0)))
                    # front view
                    # spectator = world.get_spectator()
                    # transform = hero_list[0].get_transform()
                    # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=5),
                    #                                         carla.Rotation(pitch=0, yaw=0, roll=0)))

                # ----------------------------------
                # tick and count everytime
                # ----------------------------------
                world.tick()
                global_count += 1

                # end the while loop
                if global_count >= experiment_time * 60 / settings.fixed_delta_seconds:
                    break
            else:
                world.wait_for_tick()

    finally:
        print(f'The overall collision are：speeding = {collision_counter.count(0)}; impeding = {collision_counter.count(1)}; crimping = {collision_counter.count(2)}; drunk = {collision_counter.count(3)}; distracted = {collision_counter.count(4)}')
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)


        # client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        actor_list = []
        world_snapshot = world.get_snapshot()
        for actor_snapshot in world_snapshot:
            actual_actor = world.get_actor(actor_snapshot.id)
            if 'number_of_wheels' in actual_actor.attributes:
                actor_list.append(actual_actor)
                actual_actor.destroy()

        print('\ndestroying %d vehicles' % len(actor_list))

        # # stop walker controllers (list is [controller, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
        #     all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        # for sensor in collision_sensor_list:
        #     sensor.destroy()

        time.sleep(0.5)


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

