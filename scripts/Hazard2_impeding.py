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

from carla import VehicleLightState as vls

import argparse
import logging
from numpy import random
from tools.misc_plus import get_speed, get_acceleration, get_throttle, get_brake, distance_loc, get_angle


def get_front_vehicle(map, ego_vehicle, all_vehicles, dist_threshold=20):
    for i, actor in enumerate(all_vehicles):
        if actor.id == ego_vehicle.id:
            all_vehicles.pop(i)

    target_vehicle = None
    min_dis = dist_threshold
    forward_vector = ego_vehicle.get_transform().get_forward_vector()
    for vehicle in all_vehicles:
        vehicle_transform = vehicle.get_transform()
        ego_transform = ego_vehicle.get_transform()
        wp_vehicle = map.get_waypoint(vehicle_transform.location)
        wp_ego = map.get_waypoint(ego_transform.location)
        dist = distance_loc(ego_transform.location, vehicle_transform.location)
        angle = get_angle(vehicle_transform.location, forward_vector, ego_transform.location)
        # filtering here
        if dist < dist_threshold:
            if angle < 70:
                if wp_vehicle.road_id == wp_ego.road_id and wp_vehicle.lane_id == wp_ego.lane_id:
                    if dist < min_dis:
                        min_dis = dist
                        target_vehicle = vehicle
                elif angle < 5:
                    if dist < min_dis:
                        min_dis = dist
                        target_vehicle = vehicle

    return [min_dis, target_vehicle]


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
    global dist_thd
    argparser = argparse.ArgumentParser(
        description=__doc__)
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
        default=12,
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
        default=True,
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
        default=True,
        help='Keep the spector upon the hero vehicle for debug')
    argparser.add_argument(
        '--duration', '-t',
        metavar='S',
        default=5,
        type=int,
        help='duration of the experiment (minutes)')
    argparser.add_argument(
        '--interval-factor',
        metavar='S',
        default=10,
        type=int,
        help='Sampling interval of recording data = interval_factor * fixed_delta_time')
    argparser.add_argument(
        '--R',
        metavar='S',
        default=0.5,
        type=int,
        help='ratio of impeding to the limited speed 30 km/h. Default R_imped = 1.5')
    argparser.add_argument(
        '--Dist',
        metavar='S',
        default=6,
        type=int,
        help='The distance the hero vehicle will keep from the car in front')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    random.seed(args.seed if args.seed is not None else int(time.time()))

    try:
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)

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

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        hero = args.hero
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            if hero:
                blueprint.set_attribute('role_name', 'hero')
                hero = False
            else:
                blueprint.set_attribute('role_name', 'autopilot')

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                vehicles_list.append(response.actor_id)

        # Set automatic vehicle lights update if specified
        if args.car_lights_on:
            all_vehicle_actors = world.get_actors(vehicles_list)
            for actor in all_vehicle_actors:
                traffic_manager.update_vehicle_lights(actor, True)

        # TODO configure the vehicles for experiment
        world_snapshot = world.get_snapshot()
        actor_list = []
        for actor_snapshot in world_snapshot:
            actual_actor = world.get_actor(actor_snapshot.id)
            actor_list.append(actual_actor)

        normal_list = []
        hero_list = []
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            if actor.attributes["role_name"] == 'hero':
                hero_list.append(actor)
                speed_factor = 100 * (1 - args.R)
                traffic_manager.vehicle_percentage_speed_difference(actor, speed_factor)
                traffic_manager.distance_to_leading_vehicle(actor, args.Dist)
                # traffic_manager.vehicle_lane_offset(actor, 0.5)
            else:
                normal_list.append(actor)
                if len(normal_list) >= 3:
                    break

        # Example of how to use Traffic Manager parameters
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

        # vehicle API:

        # -------------------------
        # experiment start here
        # -------------------------
        experiment_time = args.duration
        factor = args.interval_factor
        sample_time = settings.fixed_delta_seconds * factor
        dist_thd = 60

        print(f'This experiment last for {experiment_time} minutes in simulator')
        count = 0
        time_interval = np.arange(0, experiment_time*60, sample_time)
        hero_v = np.zeros_like(time_interval)
        hero_a = np.zeros_like(time_interval)
        hero_throttle = np.zeros_like(time_interval)
        hero_brake = np.zeros_like(time_interval)
        hero_dist = np.zeros_like(time_interval)

        normal_v = np.zeros((len(normal_list), len(time_interval)))
        normal_a = np.zeros((len(normal_list), len(time_interval)))
        normal_throttle = np.zeros((len(normal_list), len(time_interval)))
        normal_brake = np.zeros((len(normal_list), len(time_interval)))
        normal_dist = np.zeros((len(normal_list), len(time_interval)))
        while True:
            if not args.asynch and synchronous_master:
                world.tick()

                if args.debug:
                    # top view
                    spectator = world.get_spectator()
                    transform = hero_list[0].get_transform()
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                            carla.Rotation(pitch=-90, yaw=0)))

                # ----------------------------------
                # draw graphs
                # ----------------------------------
                if count % factor == 0:
                    index = int(count/factor)
                    for actor in hero_list:
                        hero_v[index] = get_speed(actor)
                        hero_a[index] = get_acceleration(actor)
                        hero_throttle[index] = get_throttle(actor)
                        hero_brake[index] = get_brake(actor)
                        # here should minus the length of vehicle
                        hero_dist[index] = get_front_vehicle(carla_map, actor, actor_list, dist_threshold=dist_thd)[0] - actor.bounding_box.extent.x * 2
                        # print(hero_dist[index])

                    for i, actor in enumerate(normal_list):
                        normal_v[i][index] = get_speed(actor)
                        normal_a[i][index] = get_acceleration(actor)
                        normal_throttle[i][index] = get_throttle(actor)
                        normal_brake[i][index] = get_brake(actor)
                        normal_dist[i][index] = get_front_vehicle(carla_map, actor, actor_list, dist_threshold=dist_thd)[0] - actor.bounding_box.extent.x * 2

                count += 1

                # count for experiment_time minutes
                if count >= experiment_time * 60 / settings.fixed_delta_seconds:
                    fig1, ax1 = plt.subplots()
                    fig2, ax2 = plt.subplots()
                    # fig3, ax3 = plt.subplots()
                    # fig4, ax4 = plt.subplots()
                    fig5, ax5 = plt.subplots()
                    ax1.plot(time_interval, hero_v, label='ego_vehicle')
                    for i in range(len(normal_list)):
                        ax1.plot(time_interval, normal_v[i], label='normal_' + str(i))
                    ax1.legend(loc='lower right')
                    ax1.set_xlabel('Time (s)')
                    ax1.set_ylabel('Velocity (km/h)')
                    ax1.set_title('Velocity-Time Graph')

                    ax2.plot(time_interval, hero_a, label='ego_vehicle')
                    for i in range(len(normal_list)):
                        ax2.plot(time_interval, normal_a[i], label='normal_' + str(i))
                    ax2.legend()
                    ax2.set_xlabel('Time (s)')
                    ax2.set_ylabel('Acceleration (m/s^2)')
                    ax2.set_title('Acceleration-Time Graph')

                    # ax3.plot(time_interval, hero_throttle, label='hero')
                    # for i in range(len(normal_list)):
                    #     ax3.plot(time_interval, normal_throttle[i], label='normal_' + str(i))
                    # ax3.legend()
                    # ax3.set_xlabel('Time (s)')
                    # ax3.set_ylabel('Throttle')
                    # ax3.set_title('Throttle-Time Graph')
                    #
                    # ax4.plot(time_interval, hero_brake, label='hero')
                    # for i in range(len(normal_list)):
                    #     ax4.plot(time_interval, normal_brake[i], label='normal_' + str(i))
                    # ax4.legend()
                    # ax4.set_xlabel('Time (s)')
                    # ax4.set_ylabel('Brake')
                    # ax4.set_title('Brake-Time Graph')

                    ax5.plot(time_interval, hero_dist, label='hero')
                    for i in range(len(normal_list)):
                        ax5.plot(time_interval, normal_dist[i], label='normal_' + str(i))
                    ax5.legend()
                    ax5.set_xlabel('Time (s)')
                    ax5.set_ylabel('Dist')
                    ax5.set_title('Dist-Time Graph')
                    plt.show()
                    break

            else:
                world.wait_for_tick()

    finally:
        normal_mean = []
        dis_count = []
        for vehicle in normal_dist:
            for dis in vehicle:
                if 1.5 < dis < (dist_thd - 20):
                    dis_count.append(dis)
            normal_mean.append(sum(dis_count) / len(dis_count))
        for dis in hero_dist:
            if 3 < dis < (dist_thd - 20):
                dis_count.append(dis)
        hero_mean = sum(dis_count) / len(dis_count)

        print(f"the hero mean dist : {hero_mean}; The normal list : {normal_mean}")

        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)

        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        # # stop walker controllers (list is [controller, actor, controller, actor ...])
        # for i in range(0, len(all_id), 2):
        #     all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

        time.sleep(0.5)

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')

