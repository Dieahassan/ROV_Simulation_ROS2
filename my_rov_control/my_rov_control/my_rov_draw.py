#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys
import termios
import threading


class ROV5DofController(Node):
    def __init__(self):
        super().__init__('rov_5dof_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/rov_2/controller/thruster_setpoints', 10)


        
        # --- Geometry Parameters ---
        theta = np.radians(45) # Thruster vector angle
        l_y = 0.25              # Distance from center to horizontal thrusters (m)
        dy = 0.20              # Distance from center to vertical thrusters (m)

        # Thrust Allocation Matrix (Mapping 6 Motors -> 5 DoF Forces)
        # Order: [FL, FR, VL, VR, BL, BR]
        c, s = np.cos(theta), np.sin(theta)
        self.B = np.array([
            [ c,  c,  0,   0,  c,  c],  # Fx
            [ s, -s,  0,   0, -s,  s],  # Fy
            [ 0,  0,  1,   1,  0,  0],  # Fz
            [ 0,  0, -dy, dy,  0,  0],  # Tx (Roll)
            [ l_y*s, -l_y*s, 0, 0, -l_y*s, l_y*s] # Tz (Yaw)
        ])
        
        # Calculate Pseudo-Inverse for force-to-motor allocation
        self.B_pinv = np.linalg.pinv(self.B)


    def publish_thruster_commands_from_forces(self, Fx, Fy, Fz, Tx, Tz):
        # 1. Get required 5-DoF forces
        tau = np.array([Fx, Fy, Fz, Tx, Tz])

        # 2. Map 5-DoF forces into individual thruster forces using pseudo-inverse
        thruster_forces = self.B_pinv @ tau
        
        # 3. Normalize forces between [-1.0, 1.0] for setpoint topic
        max_force = np.max(np.abs(thruster_forces))
        if max_force > 1.0:
            thruster_forces = thruster_forces / max_force

        msg = Float64MultiArray()
        msg.data = thruster_forces.tolist()
        self.publisher_.publish(msg)
    def move_circular(self, radius, speed, duration):
        """
        Moves the ROV in a circular path with given radius and speed for a specified duration.
        """
        start_time = time.time()
        while time.time() - start_time < duration:
            # Calculate the angle based on elapsed time
            elapsed_time = time.time() - start_time
            angle = (speed / radius) * elapsed_time  # Angular position

            # Calculate forces for circular motion
            Fx = speed * np.cos(angle)
            Fy = speed * np.sin(angle)
            Fz = 0.0  # No vertical movement
            Tx = 0.0  # No roll
            Tz = 0.0  # No yaw

            # Publish thruster commands based on calculated forces
            self.publish_thruster_commands_from_forces(Fx, Fy, Fz, Tx, Tz)

            time.sleep(0.1)  # Control loop delay





    


def main(args=None):
    
    rclpy.init(args=args)
    node = ROV5DofController()

    # Spin ROS node in a separate thread so key listener isn't blocked
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            # Get user input for forces and torques
            node.move_circular(radius=20.0, speed=1.0, duration=10.0)  # Example: Move in a circle of radius 1m at 0.5 m/s for 10 seconds

    except Exception as e:
        print(f"Error: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




    
    


