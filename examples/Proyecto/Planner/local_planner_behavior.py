#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module contains a local planner to perform
low-level waypoint following based on PID controllers. """

from collections import deque
from enum import Enum

import carla
from controller import VehiclePIDController
from agents.tools.misc import distance_vehicle, draw_waypoints
import math





class LocalPlanner(object):
    """
    LocalPlanner implements the basic behavior of following a trajectory
    of waypoints that is generated on-the-fly.
    The low-level motion of the vehicle is computed by using two PID controllers,
    one is used for the lateral control
    and the other for the longitudinal control (cruise speed).

    When multiple paths are available (intersections)
    this local planner makes a random choice.
    """

    # Minimum distance to target waypoint as a percentage
    # (e.g. within 80% of total distance)

    # FPS used for dt
    FPS = 20

    def __init__(self, agent):
        """
        :param agent: agent that regulates the vehicle
        :param vehicle: actor to apply to local planner logic onto
        """
        self._vehicle = agent.vehicle
        self._map = agent.vehicle.get_world().get_map()

        self._target_speed = None
        self.sampling_radius = None
        self._min_distance = None
        self._current_waypoint = None
        self.target_road_option = None
        self._next_waypoints = None
        self.target_waypoint = None
        self._vehicle_controller = None
        self._global_plan = None
        self._pid_controller = None
        self.waypoints_queue = deque(maxlen=20000)  # queue with tuples of (waypoint, RoadOption)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)
        self.prev_target_waypoint = None
        self.avoid_counter = 0
        self.prev_state = 0
        

        self._init_controller()  # initializing controller

    def reset_vehicle(self):
        """Reset the ego-vehicle"""
        self._vehicle = None
        print("Resetting ego-vehicle!")

    def _init_controller(self):
        """
        Controller initialization.

        dt -- time difference between physics control in seconds.
        This is can be fixed from server side
        using the arguments -benchmark -fps=F, since dt = 1/F

        target_speed -- desired cruise speed in km/h

        min_distance -- minimum distance to remove waypoint from queue

        lateral_dict -- dictionary of arguments to setup the lateral PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}

        longitudinal_dict -- dictionary of arguments to setup the longitudinal PID controller
                            {'K_P':, 'K_D':, 'K_I':, 'dt'}
        """
        # Default parameters
        self.args_lat_hw_dict = {
            'K_P': 0.75,
            'K_D': 0.02,
            'K_I': 0.4,
            'dt': 1.0 / self.FPS}
        self.args_lat_city_dict = {
            'K_P': 0.58,
            'K_D': 0.02,
            'K_I': 0.5,
            'dt': 1.0 / self.FPS}
        self.args_long_hw_dict = {
            'K_P': 0.37,
            'K_D': 0.024,
            'K_I': 0.032,
            'dt': 1.0 / self.FPS}
        self.args_long_city_dict = {
            'K_P': 0.15,
            'K_D': 0.05,
            'K_I': 0.07,
            'dt': 1.0 / self.FPS}

        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        self._global_plan = False

        self._target_speed = self._vehicle.get_speed_limit()

        self._min_distance = 3

    def set_speed(self, speed):
        """
        Request new target speed.

            :param speed: new target speed in km/h
        """

        self._target_speed = speed

    def set_global_plan(self, current_plan):
        """
        Sets new global plan.

            :param current_plan: list of waypoints in the actual plan
        """
        for elem in current_plan:
            self.waypoints_queue.append(elem)
        self._global_plan = True
        print("waypoints_queue[0]: ")
        print(self.waypoints_queue[0])

    def get_incoming_waypoint_and_direction(self, steps=3):
        """
        Returns direction and waypoint at a distance ahead defined by the user.

            :param steps: number of steps to get the incoming waypoint.
        """
        if len(self.waypoints_queue) > steps:
            return self.waypoints_queue[steps]

        else:
            try:
                wpt, direction = self.waypoints_queue[-1]
                print("waypoints_queue[-1]: ")
                print(self.waypoints_queue[-1])
                return wpt, direction
            except IndexError as i:
                print(i)
                return None, RoadOption.VOID
        return None, RoadOption.VOID

    def run_step(self, target_speed=None, debug=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

            :param target_speed: desired speed
            :param debug: boolean flag to activate waypoints debugging
            :return: control
        """

        if target_speed is not None:
            self._target_speed = target_speed

            if self.avoid_counter < 5:
                self._target_speed = target_speed*0.5
                self.avoid_counter += 1
                print("Reduce Speed")
                print(self.avoid_counter)
            else:
                self._target_speed = target_speed
                print("Normal Speed")


        else:
            self._target_speed = self._vehicle.get_speed_limit()

        if len(self.waypoints_queue) == 0:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False
            return control

        # Buffering the waypoints
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        # Current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        # Target waypoint
        self.target_waypoint = self._waypoint_buffer[0]
        

        if target_speed > 50:
            args_lat = self.args_lat_hw_dict
            args_long = self.args_long_hw_dict
        else:
            args_lat = self.args_lat_city_dict
            args_long = self.args_long_city_dict

        self._pid_controller = VehiclePIDController(self._vehicle,
                                                    args_lateral=args_lat,
                                                    args_longitudinal=args_long)

        control = self._pid_controller.run_step(self._target_speed, self.target_waypoint)

        # Purge the queue of obsolete waypoints
        vehicle_transform = self._vehicle.get_transform()
        max_index = -1

        for i, waypoint in enumerate(self._waypoint_buffer):
            loc = vehicle_transform.location
            x = waypoint[0] - loc.x
            y = waypoint[1] - loc.y

            distance_vehicle = math.sqrt(x * x + y * y)


            if distance_vehicle < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        if debug:
            draw_waypoints(self._vehicle.get_world(),
                           [self.target_waypoint], 1.0)
        self.prev_state = 0
        return control

    def reset_counter(self):
        self.avoid_counter = 0

    def run_avoid_step(self,heading, target_speed=None, debug=False):
            """
            Execute one step of local planning which involves
            running the longitudinal and lateral PID controllers to
            follow the waypoints trajectory.

                :param target_speed: desired speed
                :param debug: boolean flag to activate waypoints debugging
                :return: control
            """
            print(heading)

            if target_speed is not None:
                
                if self.avoid_counter < 5:
                    self._target_speed = target_speed*0.5
                    self.avoid_counter += 1
                    print("Reduce Speed")
                else:
                    self._target_speed = target_speed
                    print("Normal Speed")
                # self._target_speed = target_speed*0.5
                # if self._target_speed > 10:

                #     self._target_speed = target_speed*self.avoid_counter
                #     print("Slowing down")
                #     self.avoid_counter = self.avoid_counter + 0.1
                #     print("Ramping it up")
                # else:
                #     self._target_speed = target_speed
                #     print("Right speed")

            else:
                self._target_speed = self._vehicle.get_speed_limit()

            if len(self.waypoints_queue) == 0:
                control = carla.VehicleControl()
                control.steer = 0.0
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = False
                control.manual_gear_shift = False
                return control

            # Buffering the waypoints
            if not self._waypoint_buffer:
                for i in range(self._buffer_size):
                    if self.waypoints_queue:
                        self._waypoint_buffer.append(
                            self.waypoints_queue.popleft())
                    else:
                        break

            # Current vehicle waypoint
            self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

            # Target waypoint
            self.target_waypoint = self._waypoint_buffer[0]

            if self.target_waypoint != self.prev_target_waypoint:

                target_x = self.target_waypoint[0]
                target_y = self.target_waypoint[1]

                        #    30 < heading < 174
                if (130-90) < heading < (260-90):  #Se le restan 96° por alguna razón
                    self.target_waypoint[0] = target_x + 3.5
                    self.target_waypoint[1] = target_y + 1.11
                    print("Sur-Norte")
                elif -150 < heading < -12:
                    self.target_waypoint[0] = target_x - 3.5
                    self.target_waypoint[1] = target_y - 1.11
                    print("Norte-Sur")
                elif (-60 < heading < 25) or (10 < heading < 70):
                    self.target_waypoint[0] = target_x + 1.11
                    self.target_waypoint[1] = target_y - 3
                    print("Oeste-este")
                

                ## Método sumando de a poquito:

                # if self.avoid_counter <= 5:

                #     self.avoid_counter += 1

                #     delta_x = 0.7*self.avoid_counter
                #     delta_y = 0.222*self.avoid_counter

                #     self.target_waypoint[0] = target_x + delta_x
                #     print("Suamando ",delta_x, "a X")

                #     self.target_waypoint[1] = target_y + delta_y
                #     print("Suamando ",delta_y, "a y")
                    
            

            if target_speed > 50:
                args_lat = self.args_lat_hw_dict
                args_long = self.args_long_hw_dict
            else:
                args_lat = self.args_lat_city_dict
                args_long = self.args_long_city_dict

            self._pid_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=args_lat,
                                                        args_longitudinal=args_long)

            control = self._pid_controller.run_step(self._target_speed, self.target_waypoint)

            # Purge the queue of obsolete waypoints
            vehicle_transform = self._vehicle.get_transform()
            max_index = -1

            for i, waypoint in enumerate(self._waypoint_buffer):
                loc = vehicle_transform.location
                x = waypoint[0] - loc.x
                y = waypoint[1] - loc.y

                distance_vehicle = math.sqrt(x * x + y * y)


                if distance_vehicle < self._min_distance:
                    max_index = i
            if max_index >= 0:
                for i in range(max_index + 1):
                    self._waypoint_buffer.popleft()

            if debug:
                draw_waypoints(self._vehicle.get_world(),
                            [self.target_waypoint], 1.0)

            self.prev_target_waypoint = self.target_waypoint
            self.prev_state = 1

            return control

    def run_stop_step(self, target_speed=None, debug=False):
        """
        Execute one step of local planning which involves
        running the longitudinal and lateral PID controllers to
        follow the waypoints trajectory.

            :param target_speed: desired speed
            :param debug: boolean flag to activate waypoints debugging
            :return: control
        """
        print("Stoppin?")
        if target_speed is not None:
            self._target_speed = target_speed*0

            # if self.avoid_counter < 5:
            #     self._target_speed = target_speed*0.8
            #     self.avoid_counter += 1
            #     print("Reduce Speed")
            #     print(self.avoid_counter)
            # else:
            #     self._target_speed = target_speed
            #     print("Normal Speed")


        else:
            self._target_speed = self._vehicle.get_speed_limit()

        if len(self.waypoints_queue) == 0:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False
            return control

        # Buffering the waypoints
        if not self._waypoint_buffer:
            for i in range(self._buffer_size):
                if self.waypoints_queue:
                    self._waypoint_buffer.append(
                        self.waypoints_queue.popleft())
                else:
                    break

        # Current vehicle waypoint
        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())

        # Target waypoint
        self.target_waypoint = self._waypoint_buffer[0]
        

        if target_speed > 50:
            args_lat = self.args_lat_hw_dict
            args_long = self.args_long_hw_dict
        else:
            args_lat = self.args_lat_city_dict
            args_long = self.args_long_city_dict

        self._pid_controller = VehiclePIDController(self._vehicle,
                                                    args_lateral=args_lat,
                                                    args_longitudinal=args_long)

        control = self._pid_controller.run_step(self._target_speed, self.target_waypoint)

        # Purge the queue of obsolete waypoints
        vehicle_transform = self._vehicle.get_transform()
        max_index = -1

        for i, waypoint in enumerate(self._waypoint_buffer):
            loc = vehicle_transform.location
            x = waypoint[0] - loc.x
            y = waypoint[1] - loc.y

            distance_vehicle = math.sqrt(x * x + y * y)


            if distance_vehicle < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        if debug:
            draw_waypoints(self._vehicle.get_world(),
                           [self.target_waypoint], 1.0)
        self.prev_state = 0
        return control