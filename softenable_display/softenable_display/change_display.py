import time
import threading
import rclpy
from rclpy.node import Node
from softenable_display_msgs.srv import SetDisplay
from std_srvs.srv import Trigger

class SetDisplaySwitcher(Node):
    def __init__(self):
        super().__init__('set_display_switcher')

        # Client to SetDisplay
        self.cli_display = self.create_client(SetDisplay, '/set_display')
        while not self.cli_display.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_display service...')

        # Service to start switching
        self.srv_start = self.create_service(Trigger, 'start_switch', self.handle_start_switch)
        # Kill switch service
        self.srv_kill = self.create_service(Trigger, 'kill_switch', self.handle_kill_switch)

        # State
        self.switch_thread = None
        self.stop_requested = threading.Event()

        self.get_logger().info("Ready. Call /start_switch to start or /kill_switch to stop switching.")

    # ----------- Start service callback -----------
    def handle_start_switch(self, request, response):
        """Start switching thread (stop previous if running)."""
        if self.switch_thread and self.switch_thread.is_alive():
            self.get_logger().warn("Previous switching still running. Stopping it first...")
            self.stop_requested.set()
            self.switch_thread.join(timeout=3.0)
            if self.switch_thread.is_alive():
                self.get_logger().warn("Previous thread did not stop in time. Forcing restart.")

        # Start new thread
        self.stop_requested.clear()
        self.switch_thread = threading.Thread(target=self.switch_displays, daemon=True)
        self.switch_thread.start()

        response.success = True
        response.message = "Started display switching (previous one stopped if running)."
        return response

    # ----------- Kill switch callback -----------
    def handle_kill_switch(self, request, response):
        """Immediately stop the switching thread if it's running."""
        if self.switch_thread and self.switch_thread.is_alive():
            self.get_logger().warn("Kill switch activated. Stopping thread NOW...")
            self.stop_requested.set()
            self.switch_thread.join(timeout=2.0)
            if self.switch_thread.is_alive():
                self.get_logger().error("Thread did not stop within timeout — may still be running!")
                response.success = False
                response.message = "Thread did not fully stop."
            else:
                self.get_logger().info("Switching thread successfully stopped.")
                response.success = True
                response.message = "Switching stopped."
        else:
            response.success = True
            response.message = "No switching thread running."
        return response

    # ----------- Worker thread -----------
    def switch_displays(self):
        for i in range(1, 8):
            if self.stop_requested.is_set():
                self.get_logger().info("Switching aborted.")
                return

            preset = f"protocol_{i}"
            self.get_logger().info(f"Applying preset {preset}")

            req = SetDisplay.Request()
            req.name = preset
            req.use_tts = True

            future = self.cli_display.call_async(req)
            future.add_done_callback(lambda fut: self.get_logger().info(f"Preset '{preset}' done"))

            sleep_sec = 3 if i == 1 else 7
            for _ in range(sleep_sec):
                if self.stop_requested.is_set():
                    self.get_logger().info("Switching aborted during sleep.")
                    return
                time.sleep(1)

        self.get_logger().info("Finished switching displays.")

def main(args=None):
    rclpy.init(args=args)
    node = SetDisplaySwitcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_requested.set()
        if node.switch_thread and node.switch_thread.is_alive():
            node.switch_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
