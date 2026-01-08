#!/usr/bin/env python3
"""
Festo Gripper Controller Node
- Listens to /robot/gripper_cmd topic from C++ robot_logic_node
- Controls Festo CPX-AP gripper valve via CPX IO module
- No 'rich' dependency required
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

# Suppress rich dependency warning
import warnings
warnings.filterwarnings('ignore', message='.*rich.*')

try:
    from cpx_io.cpx_system.cpx_ap.cpx_ap import CpxAp
    CPX_AVAILABLE = True
except ImportError as e:
    CPX_AVAILABLE = False
    print(f"⚠️  Warning: festo-cpx-io not available: {e}")
    print("   Running in simulation mode (commands will be logged only)")


class FestoGripperNode(Node):
    def __init__(self):
        super().__init__('festo_gripper_controller')
        
        # Get CPX IP from parameters (default: Festo gripper IP)
        self.declare_parameter('cpx_ip', '192.168.27.163')
        self.cpx_ip = self.get_parameter('cpx_ip').value
        
        # Get module index (default: module 1)
        self.declare_parameter('cpx_module_index', 1)
        self.cpx_module_index = self.get_parameter('cpx_module_index').value
        
        # Simulation mode flag
        self.simulation_mode = not CPX_AVAILABLE
        
        self.get_logger().info(f'Connecting to Festo CPX at {self.cpx_ip}...')
        
        # Initialize CPX connection (degraded mode allowed)
        self.myCPX = None
        self.myIO = None
        
        if CPX_AVAILABLE:
            try:
                self.myCPX = CpxAp(ip_address=self.cpx_ip)
                # Validate module index
                if self.cpx_module_index < len(self.myCPX.modules):
                    self.myIO = self.myCPX.modules[self.cpx_module_index]
                    # single logical name for module
                    self.myIO.name = "festo_io_module"
                    self.get_logger().info(f'✔ Connected to Festo CPX module {self.cpx_module_index}')
                else:
                    self.get_logger().error(f'✗ CPX module index {self.cpx_module_index} out of range')
                    self.myIO = None
                    self.simulation_mode = True
            except Exception as e:
                self.get_logger().error(f'✗ Failed to connect to Festo CPX: {e}')
                self.get_logger().warn('⚠️  Running in SIMULATION MODE')
                self.simulation_mode = True
        else:
            self.get_logger().warn('⚠️  CPX library not available - Running in SIMULATION MODE')

        # State flags for gripper and picker (separate devices/channels)
        self.gripper_open = True
        self.picker_open = True
        
        # Subscriptions from robot_logic_node
        self.gripper_sub = self.create_subscription(
            Bool,
            '/robot/gripper_cmd',
            self.gripper_callback,
            10
        )

        # Optional picker commands (separate channels)
        self.picker_sub = self.create_subscription(
            Bool,
            '/robot/picker_cmd',
            self.picker_callback,
            10
        )

        mode_str = "SIMULATION" if self.simulation_mode else "LIVE"
        self.get_logger().info(f'[{mode_str}] Waiting for gripper commands on /robot/gripper_cmd and /robot/picker_cmd...')
    
    def gripper_callback(self, msg: Bool):
        """
        Callback when gripper command received
        msg.data = True  → Gripper ON (close/grip)
        msg.data = False → Gripper OFF (open/release)
        """
        try:
            if msg.data:
                # Gripper ON: Close (grip)
                self.gripper_close()
            else:
                # Gripper OFF: Open (release)
                self.gripper_open_cmd()
        except Exception as e:
            self.get_logger().error(f'Error controlling gripper: {e}')


    def gripper_close(self):    #gripper_close 0        
        """Close gripper - equivalent to setting valve"""
        if self.gripper_open:
            self.get_logger().info('🔽 Gripper: CLOSING (setting valve)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Gripper closed (channels: 0=reset, 1=set)')
                self.gripper_open = False
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                # Use channel 0 = open, 1 = close (module-specific mapping)
                self.myIO.reset_channel(0)
                self.myIO.set_channel(1)
                self.gripper_open = False
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f'Failed to close gripper: {e}')
    
    def gripper_open_cmd(self):      #gripper_open_cmd 1
        """Open gripper - reset valve"""
        if not self.gripper_open:
            self.get_logger().info('🔼 Gripper: OPENING (resetting valve)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Gripper opened (channels: 1=reset, 0=set)')
                self.gripper_open = True
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                # Reset close channel then set open
                self.myIO.reset_channel(1)
                self.myIO.set_channel(0)
                time.sleep(0.05)
                self.gripper_open = True
            except Exception as e:
                self.get_logger().error(f'Failed to open gripper: {e}')

    # Picker methods (previously defined outside the class) - use channels 2/3
    def picker_callback(self, msg: Bool):
        """Callback for picker commands (separate from gripper)"""
        try:
            if msg.data:
                self.picker_close()
            else:
                self.picker_open_cmd()
        except Exception as e:
            self.get_logger().error(f'Error controlling picker: {e}')

    def picker_close(self):
        """Close picker - set channels for picker close"""
        if self.picker_open:
            self.get_logger().info('🔽 Picker: CLOSING (setting valve)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Picker closed (channels: 2=reset, 3=set)')
                self.picker_open = False
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                self.myIO.reset_channel(2)
                self.myIO.set_channel(3)
                self.picker_open = False
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f'Failed to close picker: {e}')

    def picker_open_cmd(self):
        """Open picker - reset picker channels"""
        if not self.picker_open:
            self.get_logger().info('🔼 Picker: OPENING (resetting valve)')
            
            if self.simulation_mode:
                self.get_logger().info('[SIM] Picker opened (channels: 3=reset, 2=set)')
                self.picker_open = True
                return
            
            try:
                if not self.myIO:
                    raise RuntimeError('CPX IO not available')
                self.myIO.reset_channel(3)
                self.myIO.set_channel(2)
                time.sleep(0.05)
                self.picker_open = True
            except Exception as e:
                self.get_logger().error(f'Failed to open picker: {e}')
    

 
    def shutdown(self):
        """Cleanup on shutdown"""
        try:
            self.get_logger().info('Shutting down Festo gripper controller...')
            
            if self.simulation_mode:
                self.get_logger().info('✔ Simulation mode shutdown complete')
                return
            
            # Try to open both before shutdown if possible
            try:
                self.gripper_open_cmd()
            except Exception:
                pass
            try:
                self.picker_open_cmd()
            except Exception:
                pass
            if self.myCPX:
                try:
                    self.myCPX.close()
                except Exception:
                    pass
            self.get_logger().info('✔ Festo CPX connection closed')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FestoGripperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutdown requested')
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()