import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import logging
from carla import *


def main():
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    walkers_list = []
    all_id = []
    # 0. Choose a blueprint fo the walkers
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    blueprintsWalkers = world.get_blueprint_library().filter("walker.pedestrian.*")
    walker_bp = random.choice(blueprintsWalkers)

    # 1. Take all the random locations to spawn
    spawn_points = []

    x_points = [-195.156, -189.520, -182.170, -198.341, -204.221]
    y_points = [262.195, 238.919, 214.662, 220.543, 242.349]
    x_points_F = []
    y_points_F = []
    for i in range(5):
        x_points_F.append(-212.903)
        y_points_F.append(323.576)


    for i in range(5):
        spawn_point = carla.Transform()
        spawn_point = Transform(Location(x= x_points[i], y=y_points[i] ,z=2 ),Rotation (yaw=0))
        spawn_points.append(spawn_point)

    # 2. Build the batch of commands to spawn the pedestrians
    batch = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
        print("Spawn Succesful")

    # 2.1 apply the batch
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})

    # 3. Spawn walker AI controllers for each walker
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        print("Controlador spawneado en el walker ",i)

    # 3.1 apply the batch
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
        print("Batch aplicado al walker ",i)

    # 4. Put altogether the walker and controller ids
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
        print("Putted altogether el id de walker ", i)
    all_actors = world.get_actors(all_id)

    # wait for a tick to ensure client receives the last transform of the walkers we have just created
    world.wait_for_tick()

    # 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
    for i in range(0, len(all_actors), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())

        # random max speed
        all_actors[i].set_max_speed(1 + random.random())    # max speed between 1 and 2 (default is 1.4 m/s)

    time.sleep(20)

    for i in range(0, len(all_id), 2):
        all_actors[i].stop()

    # destroy pedestrian (actor and controller)
    client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

if __name__ == '__main__':

    main()