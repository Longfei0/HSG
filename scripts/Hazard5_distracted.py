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
from tools.misc_plus import get_speed, get_acceleration, get_throttle, get_brake, get_steer


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
        '--opt',
        action='store_true',
        default=False,
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
    random.seed(args.seed if args.seed is not None else int(time.time()))

    # remove all the static vehicles first
    # print(client.get_available_maps())
    if args.map is not None and args.opt is not None:
        print('load map %r.' % args.map)
        world = client.load_world(args.map, map_layers=carla.MapLayer.NONE)
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
        random.shuffle(spawn_points)
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
        normal_list = []
        hero_list = []
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            if actor.attributes["role_name"] == 'hero':
                hero_list.append(actor)
                # traffic_manager.ignore_lights_percentage(actor, 50)
                # traffic_manager.ignore_signs_percentage(actor, 50)
                # traffic_manager.set_desired_speed(actor, )
                # traffic_manager.vehicle_percentage_speed_difference(actor, 30)
                # traffic_manager.vehicle_lane_offset(actor, 0.5)
            else:
                normal_list.append(actor)
                if len(normal_list) >= 3:
                    break

        # Example of how to use Traffic Manager parameters
        traffic_manager.global_percentage_speed_difference(0)
        traffic_manager.set_global_distance_to_leading_vehicle(2.5)
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
        count = 0
        sample_time = settings.fixed_delta_seconds * factor
        count_same_flag = False
        next_speed_time = count
        ControlLoss_interval = args.t_interval  # in seconds (int)
        controloss_start = []  # here to save the counts when the control loss starts
        controloss_end = []  # here to save the counts when the control loss ends

        indexs = int(experiment_time * 60 / ControlLoss_interval)
        for i in range(indexs):
            # t_last = 10
            t_last = np.clip(random.normal(2.5, 1), args.t_loss[0], args.t_loss[1])
            t_start = random.uniform(0+t_last, ControlLoss_interval-t_last)
            start = int(t_start/settings.fixed_delta_seconds) + i * ControlLoss_interval/settings.fixed_delta_seconds
            end = start + int(t_last/settings.fixed_delta_seconds)
            controloss_start.append(start)
            controloss_end.append(end)
        # this is to make sure the list won't out of range
        controloss_start.extend([0, 0])
        controloss_end.extend([0, 0])

        # -------------------------
        # graph initializing
        # -------------------------
        print(f'This experiment last for {experiment_time} minutes in simulator')
        time_interval = np.arange(0, experiment_time * 60, sample_time)
        hero_v = np.zeros_like(time_interval)
        hero_a = np.zeros_like(time_interval)
        hero_throttle = np.zeros_like(time_interval)
        hero_brake = np.zeros_like(time_interval)
        hero_steering = np.zeros_like(time_interval)
        hero_same_throttle = -1 * np.ones_like(time_interval)

        normal_v = np.zeros((len(normal_list), len(time_interval)))
        normal_a = np.zeros((len(normal_list), len(time_interval)))
        normal_throttle = np.zeros((len(normal_list), len(time_interval)))
        normal_brake = np.zeros((len(normal_list), len(time_interval)))
        while True:
            if not args.asynch and synchronous_master:
                world.tick()

                for actor in hero_list:
                    if controloss_start[0] == count:
                        actor.set_autopilot(False, traffic_manager.get_port())
                        control = actor.get_control()
                        t = round((controloss_end[0]-controloss_start[0]) * settings.fixed_delta_seconds, 2)
                        print(f'I am gonna lose control for {t} seconds')
                        count_same_flag = True
                    elif controloss_start[0] < count < controloss_end[0]:
                        actor.apply_control(control)
                        fix_box = carla.BoundingBox(actor.get_transform().location + carla.Location(
                            z=actor.bounding_box.extent.z), actor.bounding_box.extent)
                        world.debug.draw_box(fix_box, actor.get_transform().rotation, 0.05, carla.Color(100, 100, 0), 0.15)

                    elif controloss_end[0] == count:
                        count_same_flag = False
                        controloss_start.pop(0)
                        controloss_end.pop(0)
                        print('control recovered')
                        actor.set_autopilot(True, traffic_manager.get_port())

                # if count == next_speed_time:
                #     speed = np.clip(random.normal(0, 25), -50, 50)
                #     traffic_manager.vehicle_percentage_speed_difference(hero_list[0], speed)
                #     # set the offset distance reset time for the next time
                #     next_speed_time = count + int(random.uniform(4, 10) / settings.fixed_delta_seconds)

                # detect whether running a red light:
                # if is_running_RedLight():
                #     stop_flag = True

                if args.debug:
                    # top view
                    spectator = world.get_spectator()
                    transform = hero_list[0].get_transform()
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                            carla.Rotation(pitch=-90, yaw=0)))
                    # front view
                    # spectator = world.get_spectator()
                    # transform = hero_list[0].get_transform()
                    # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=5),
                    #                                         carla.Rotation(pitch=0, yaw=0, roll=0)))

                # ----------------------------------
                # draw graphs
                # ----------------------------------
                if count % factor == 0:
                    index = int(count / factor)
                    for actor in hero_list:
                        hero_v[index] = get_speed(actor)
                        hero_a[index] = get_acceleration(actor)
                        hero_throttle[index] = get_throttle(actor)
                        hero_brake[index] = get_brake(actor)
                        hero_steering[index] = get_steer(actor)
                        if count_same_flag:
                            hero_same_throttle[index] = get_throttle(actor)
                    for i, actor in enumerate(normal_list):
                        normal_v[i][index] = get_speed(actor)
                        normal_a[i][index] = get_acceleration(actor)
                        normal_throttle[i][index] = get_throttle(actor)
                        normal_brake[i][index] = get_brake(actor)
                count += 1
                # count for experiment_time minutes
                if count >= experiment_time * 60 / settings.fixed_delta_seconds:
                    # fig1, ax1 = plt.subplots()
                    # fig2, ax2 = plt.subplots()
                    fig3, ax3 = plt.subplots()
                    fig4, ax4 = plt.subplots()
                    fig5, ax5 = plt.subplots()
                    # ax1.plot(time_interval, hero_v, label='hero')
                    # for i in range(len(normal_list)):
                    #     ax1.plot(time_interval, normal_v[i], label='normal_' + str(i))
                    # ax1.legend()
                    # ax1.set_xlabel('Time (s)')
                    # ax1.set_ylabel('Velocity (m/s)')
                    # ax1.set_title('Velocity-Time Graph')
                    #
                    # ax2.plot(time_interval, hero_a, label='hero')
                    # for i in range(len(normal_list)):
                    #     ax2.plot(time_interval, normal_a[i], label='normal_' + str(i))
                    # ax2.legend()
                    # ax2.set_xlabel('Time (s)')
                    # ax2.set_ylabel('Acceleration (m/s^2)')
                    # ax2.set_title('Acceleration-Time Graph')

                    ax3.plot(time_interval, hero_throttle, 'b-', label='ego_vehicle')
                    start_idx = None
                    for idx, value in enumerate(hero_same_throttle):
                        if value != -1 and start_idx is None:
                            start_idx = idx
                        elif value == -1 and start_idx is not None:
                            ax3.plot(time_interval[start_idx:idx], hero_same_throttle[start_idx:idx], 'r-', linewidth=4.5,
                                     label="Control_loss" if start_idx == 3 else None)
                            start_idx = None
                    if start_idx is not None:
                        ax3.plot(time_interval[start_idx:], hero_same_throttle[start_idx:], 'r-', linewidth=4.5, label="Control_Loss")
                    # ax3.plot(time_interval, hero_same_throttle, 'r-', label="Control_Loss")
                    ax3.legend()
                    ax3.set_xlabel('Time (s)')
                    ax3.set_ylabel('Throttle')
                    ax3.set_title('Throttle-Time Graph')

                    ax4.plot(time_interval, hero_brake, label='ego_control_loss')
                    # for i in range(len(normal_list)):
                    #     ax4.plot(time_interval, normal_brake[i], label='normal_' + str(i))
                    ax4.legend()
                    ax4.set_xlabel('Time (s)')
                    ax4.set_ylabel('Brake')
                    ax4.set_title('Brake-Time Graph')

                    ax5.plot(time_interval, hero_steering, label='hero_control_loss')
                    # for i in range(len(normal_list)):
                    #     ax5.plot(time_interval, normal_brake[i], label='normal_' + str(i))
                    ax5.legend()
                    ax5.set_xlabel('Time (s)')
                    ax5.set_ylabel('Steer')
                    ax5.set_title('Steer-Time Graph')

                    plt.show()

                    break

            else:
                world.wait_for_tick()

    finally:

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

