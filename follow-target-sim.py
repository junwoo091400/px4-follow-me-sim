#!/usr/bin/env python3

import os
import time
import math
import argparse
import asyncio

from mavsdk import System, param
from mavsdk.follow_me import (Config, FollowMeError, TargetLocation)

from target_model import simulation


async def fly_drone(model, responsiveness, pub_rc, spam_gps, no_takeoff):
    # Get starting location from env variables
    # This should match the coordinates used by gazebo or jmavsim, which load
    # the same env variables
    PX4_HOME_LAT = float(os.getenv('PX4_HOME_LAT', default=47.397742))
    PX4_HOME_LON = float(os.getenv('PX4_HOME_LON', default=8.545594))
    PX4_HOME_ALT = float(os.getenv('PX4_HOME_ALT', default=488))

    default_height = 8.0  # in Meters
    # in Meters, this is the distance that the drone will remain away from
    # Target while following it
    follow_distance = 8.0

    # Direction relative to the Target
    # Options are NONE, FRONT, FRONT_LEFT, FRONT_RIGHT, BEHIND
    direction = Config.FollowDirection.BEHIND

    drone = System()
    await drone.connect(system_address="udp://:14540")

    # This waits till a mavlink based drone is connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"--- Connected to drone!")
            break

    print("--- Setting parameters...")
    # await drone.param.set_param_int('NAV_RCL_ACT', 0) # Disable datalink failsafe or simulation won't work without QGC open
    # await drone.param.set_param_int('COM_RC_IN_MODE', 4) # Disable stick input
    await drone.param.set_param_int('NAV_DLL_ACT', 0) # Set data link loss failsafe mode to 'Disabled'
    # await drone.param.set_param_float('COM_RC_LOSS_T', 1.5)
    # await drone.param.set_param_int('COM_DL_LOSS_T', 10)

    # Checking if Global Position Estimate is ok
    print("--- Waiting for GPS")
    async for global_lock in drone.telemetry.health():
        if global_lock.is_global_position_ok and global_lock.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    dt = 0.01
    t = 0.0
    t_max = 100
    time_start = time.time()
    last_update = 0

    if model == 'circle':
        target_model = simulation.TargetModelCircle(
            PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT)
    elif model == "line":
        target_model = simulation.TargetModelStraightLine(
            PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT)
    elif model == "rectangle":
        target_model = simulation.TargetModelRectangle(
            PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT)
    elif model == "point":
        target_model = simulation.TargetModelPoint(
            PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT)
    elif model == "unitspeed":
        target_model = simulation.TargetModelUnitSpeed(
            PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT)
    elif model == "go_and_stop":
        target_model = simulation.TargetModelGoAndStop(
            PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT)
    else:
        # Not a valid model, stop the program
        print("!!!! {} is not a valid model! Please choose a correct model. Exiting program ...".format(model))
        return

    # Simulation loop for target model
    armed = False
    takeoff = False
    mode_switched = False
    configured = False
    target_location = None
    rtl_stop_time = None
    land_time = None

    if no_takeoff:
        takeoff = True
        land_time = 1

    while(True):
        t = time.time() - time_start
        time_loop_start = time.time()

        # Send manual control input with all sticks centered
        if pub_rc:
            await drone.manual_control.set_manual_control_input(0.0, 0.0, 0.5, 0.0)

        if not armed:
            # Arming the drone
            print("-- Arming")
            await drone.action.arm()
            armed = True

        # if not configured:
        #     # Follow me Mode requires some configuration to be done before
        #     # starting the mode
        #     conf = Config(
        #         default_height,
        #         follow_distance,
        #         direction,
        #         responsiveness)
        #     await drone.follow_me.set_config(conf)
        #     configured = True

        if not takeoff:
            print("-- Taking Off")
            await drone.action.takeoff()
            takeoff = True

        # Wait 8 seconds with next step
        if not mode_switched and t > 8.0:
            print("-- Starting Follow Me Mode")
            await drone.follow_me.start()
            mode_switched = True

        # Wait 4 seconds with next step
        if t > 12.0 and t < t_max:
            # Update target model with every timestep
            target_model.update(t - 12.0)

            # Send a position update at 1 Hz
            if (time_loop_start - last_update >= 1):
                last_update = time_loop_start

                # Update model
                target_location = TargetLocation(
                    target_model.lat,
                    target_model.lon,
                    PX4_HOME_ALT,
                    target_model.vx,
                    target_model.vy,
                    0)

                if not spam_gps:
                    print("-- Sending Following Target for t=" + str(round(t, 2)))
                    print(target_location)
                    print("")
                    await drone.follow_me.set_target_location(target_location)

            # Send position updated much more often than at 1 Hz. This results in duplicate messages, good for testing
            # This is similar to iOS and Android running QGC, where the GPS position typically updates at 1 Hz, but
            # messages are still send at higher rates such as 10 Hz with slightly varying velocity estimates. Probably
            # because the phones' estimators run at a higher rate than 1 Hz as
            # well in order to provide a smoother user experience.
            if spam_gps and target_location is not None:
                print("-- Sending Following Target for t=" + str(round(t, 2)))
                print(target_location)
                print("")
                await drone.follow_me.set_target_location(target_location)

        # Final phase performing RTL / landing
        if t > t_max:
            # Stopping the follow me mode
            if not rtl_stop_time:
                print("-- Stopping Follow Me Mode")
                await drone.follow_me.stop()
                rtl_stop_time = t

            if t > rtl_stop_time + 5 and land_time is None:
                print("-- Landing")
                await drone.action.return_to_launch()
                land_time = t

        # Update time
        measured_dt = time.time() - time_loop_start
        sleep_duration = dt - measured_dt
        if sleep_duration > 0:
            await asyncio.sleep(sleep_duration)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='PX4 follow-me target simulator')
    parser.add_argument(
        '--responsiveness',
        type=float,
        required=False,
        default=0.1,
        help='Responsiveness parameter for follow-me in range [0.0, 1.0]')
    parser.add_argument(
        '--model',
        type=str,
        required=False,
        help='motion model for target, "circle" or "line"',
        default="circle")
    parser.add_argument(
        '--no-rc',
        dest='rc',
        action='store_false',
        help='Do not send dummy RC commands')
    parser.set_defaults(rc=True)
    parser.add_argument('--spam_gps', type=bool, required=False,
                        help='Spam GPS messages at high rates', default=False)
    parser.add_argument(
        '--no_takeoff',
        action='store_true'
    )

    args = parser.parse_args()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(
        fly_drone(
            args.model,
            args.responsiveness,
            args.rc,
            args.spam_gps,
            args.no_takeoff))
