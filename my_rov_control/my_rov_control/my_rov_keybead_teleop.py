#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys
import select
import termios
import tty
import threading

instructions = """
==================================================
        ROV 5-DoF (6 Motors) Keyboard Teleop      
==================================================
  [W] / [S] : Forward / Backward (Surge - Fx)
  [A] / [D] : Sway Left / Right  (Sway  - Fy)
  [R] / [F] : Move Up / Down     (Heave - Fz)

  [Q] / [E] : Yaw Left / Right   (Yaw   - Tz)
  [J] / [L] : Roll Left / Right  (Roll  - Tx)

  [SPACE]   : Stop All Motors
  [CTRL-C]  : Quit
==================================================
"""

def getKey(settings):
    """Reads a single keypress from standard input without requiring Enter."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class ROV5DofController(Node):
    def __init__(self):
        super().__init__('rov_5dof_controller')

        # Topic for sending setpoints to Stonefish / ROV simulation
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/rov_2/controller/thruster_setpoints', 
            10
        )

        # Output speed factor [0.0 to 1.0]
        self.power = 1.0
    def publish_thruster_commands(self, F_1,F_2,F_3,F_4,F_5,F_6):
            """
            Mixes 5 DoF commands into 6 motor setpoints [-1.0, 1.0]
            Motors Order: [FL, FR, BL, BR, VL, VR]
            """
            # Horizontal thrust allocation (Vectored 45 degrees)
    
    
            thruster_setpoints = np.array([F_1,F_2,F_3,F_4,F_5,F_6]) * self.power
    
    
            msg = Float64MultiArray()
            msg.data = thruster_setpoints.tolist()
            print(msg.data)
            self.publisher_.publish(msg)
            

    def publish_thruster_commands_1(self, Fx, Fy, Fz, Tx, Tz):
        """
        Mixes 5 DoF commands into 6 motor setpoints [-1.0, 1.0]
        Motors Order: [FL, FR, BL, BR, VL, VR]
        """
        vel=1.0
        # Horizontal thrust allocation (Vectored 45 degrees)
        t_fl =  Fx + Fy -Tx
        t_fr =  Fx - Fy +Tx
        t_bl =  Fx - Fy -Tx
        t_br =  Fx + Fy +Tx

        # Vertical thrust allocation (Heave + Roll)
        t_vl =  Fz +Tz
        t_vr =  Fz -Tz

        thruster_setpoints = np.array([t_fl, vel*t_fr, t_vl, vel*t_vr, t_bl, vel*t_br]) * self.power    


        msg = Float64MultiArray()
        msg.data = thruster_setpoints.tolist()
        self.publisher_.publish(msg)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ROV5DofController()

    # Spin ROS node in a separate thread so key listener isn't blocked
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(instructions)

    try:
        while True:
            key = getKey(settings)

            Fx, Fy, Fz = 0.0, 0.0, 0.0
            Tx, Tz = 0.0, 0.0
            VEL=-1000.0
            F_1, F_2, F_3 = 0.0, 0.0, 0.0
            F_4, F_5,F_6 = 0.0, 0.0,0.0

            if key == 'w':
                Fx = 1.0
            elif key == 's':
                Fx = -1.0
            elif key == 'a':
                Fy = 1.0
            elif key == 'd':
                Fy = -1.0
            elif key == 'r':
                Fz = 1.0
            elif key == 'f':
                Fz = -1.0
            elif key == 'q':
                Tz = 1.0
            elif key == 'e':
                Tz = -1.0
            elif key == 'j':
                Tx = 1.0/2
            elif key == 'l':
                Tx = -1.0/2
            elif key == ' ':
                Fx, Fy, Fz, Tx, Tz = 0.0, 0.0, 0.0, 0.0, 0.0
                F_1,F_2,F_3,F_4,F_5,F_6 = 0.0, 0.0, 0.0, 0.0, 0.0,0.0


            elif key == '7':
                F_1 = 1.0
            elif key == '9':
                F_2 = VEL
            elif key == '4':
                F_3 = 1.0
            elif key == '6':
                F_4 = VEL
            elif key == '1':
                F_5 = 1.0
            elif key == '3':
                F_6 = VEL
            elif key == '5':
                F_1,F_2,F_3,F_4,F_5,F_6=1.0,VEL,1.0,VEL,1.0,VEL
            
            elif key == '\x03':  # Ctrl+C
                break
            if F_1 or F_2 or F_3 or F_4 or F_5 or F_6:
                node.publish_thruster_commands(F_1,F_2,F_3,F_4,F_5,F_6)
            else:
                node.publish_thruster_commands_1(Fx, Fy, Fz, Tx, Tz)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Zero all motors on exit and restore terminal state
        node.publish_thruster_commands(0.0, 0.0, 0.0, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




    
    
    