#!/usr/bin/env python3

import asyncio
from math import *
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
import csv

async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    with open("points.csv", mode="r", newline="") as file:
        reader = csv.DictReader(file)
        
        for rows in reader:
            x = float(rows["x"])
            y = float(rows["y"])
            # z = float(rows["z"])
            dist = float(rows["dist"])
            
            await drone.offboard.set_position_ned(
                PositionNedYaw(x,y,-1.0,0.0))
            print(x,y,dist)
            await asyncio.sleep(0.1)


    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())