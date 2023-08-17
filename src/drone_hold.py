#!/usr/bin/env python3

import os
import rospy
from ros_basics_tutorials.msg import samplemsg
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

class drone_hold():
    def __init__(self):
        rospy.init_node('drone_hold', anonymous=True)
        rospy.Rate(10)
        self.command_sub = rospy.Subscriber("/command_topic",samplemsg,self.posecallback)
        self.command = None

    def posecallback(self,msg):
        self.command = msg.name

    async def run(self):

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

        x = 1.0
        y = 0.0

        await drone.offboard.set_position_ned(
                PositionNedYaw(x, y, -2.0, 90.0))
        count = 0
        while count < 1000:
            
            if self.command == "Forward":
                print(f"Going {self.command} by 0.2")
                y += 0.2
                await drone.offboard.set_position_ned(
                PositionNedYaw(x, y, -2.0, 90.0))
                await asyncio.sleep(0.5)
                self.command = None
            if self.command == "Backward":
                print(f"Going {self.command} by 0.2")
                y -= 0.2
                await drone.offboard.set_position_ned(
                PositionNedYaw(x, y, -2.0, 90.0))
                await asyncio.sleep(0.5)
                self.command = None
            if self.command == "Left":
                print(f"Going {self.command} by 0.2")
                x += 0.2
                await drone.offboard.set_position_ned(
                PositionNedYaw(x, y, -2.0, 90.0))
                await asyncio.sleep(0.5)
                self.command = None
            if self.command == "Right":
                print(f"Going {self.command} by 0.2")
                x -= 0.2
                await drone.offboard.set_position_ned(
                PositionNedYaw(x, y, -2.0, 90.0))
                await asyncio.sleep(0.5)
                self.command = None
            else:
                await drone.offboard.set_position_ned(
                PositionNedYaw(x, y, -2.0, 90.0))
                await asyncio.sleep(0.2)
                
            
        # os.system("python try.py")
        # os.system("python centroid_track.py")

        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed \
                    with error code: {error._result.result}")

if __name__ == "__main__":
    dh = drone_hold()
    asyncio.run(dh.run())