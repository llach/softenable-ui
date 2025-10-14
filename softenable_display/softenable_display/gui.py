import os
import sys
import time
import rclpy
import threading

from PyQt5 import QtWidgets, uic  # or PyQt6 if you prefer
from PyQt5.QtCore import QThreadPool, QRunnable, pyqtSlot, QTimer
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from std_srvs.srv import Trigger
from stack_msgs.srv import RollerGripper

class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    @pyqtSlot()
    def run(self):
        self.fn(*self.args, **self.kwargs)

class ControlNode(Node):
    """ROS 2 Node that handles service calls and communication."""

    def __init__(self):
        super().__init__("control_panel_node")

        # create service clients
        self.gripper_l_cli = self.create_client(RollerGripper, "left_roller_gripper")
        self.gripper_r_cli = self.create_client(RollerGripper, "right_roller_gripper")

        self.switch_cli = self.create_client(Trigger, "start_switch")

    def open_gripper(self, side: str, open: bool):
        """Example service call."""
        cli = self.gripper_l_cli if side == "left" else self.gripper_r_cli
        if not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{side} gripper service not available")
            return

        req = RollerGripper.Request()
        req.open = open

        future = cli.call_async(req)
        # you can add a done callback if you want:
        future.add_done_callback(lambda f: print(f"[{side}] Response: {f.result()}"))

    def test_srv(self):
        time.sleep(5)
        fut = self.switch_cli.call_async(Trigger.Request())
        self.wait_for_futures([fut])

    def wait_for_futures(self, futures):
        for f in futures: rclpy.spin_until_future_complete(self, f)

class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        ##### ROS Setup
        rclpy.init(args=None)
        self.node = ControlNode()

        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.ros_thread.start()

        ##### Load UI
        pkg_share = get_package_share_directory('softenable_display')
        uic.loadUi(os.path.join(pkg_share, 'main_window.ui'), self)

        self.pool = QThreadPool()

        ##### Button setup
        self.btnBagDemo.clicked.connect(lambda: self.run_with_disable(self.btnBagDemo, self.node.test_srv))
        self.btnBagOpen.clicked.connect(lambda: self.action("Bag Opening Open"))
        self.btnBagRetreat.clicked.connect(lambda: self.action("Bag Opening Retreat & Slides"))

        self.btnUnstackDemo.clicked.connect(lambda: self.action("Unstacking Demo"))
        self.btnUnstackSlides.clicked.connect(lambda: self.action("Unstacking Slides"))

        self.btnUnfold.clicked.connect(lambda: self.action("Unfolding Unfold"))

        self.btnToolsOpen.clicked.connect(lambda: self.action("Tools Open"))
        self.btnToolsClose.clicked.connect(lambda: self.action("Tools Close"))
        self.btnFinalSlides.clicked.connect(lambda: self.action("Final Slides"))

    def run_with_disable(self, button, func, *args, **kwargs):

        def wrapped():
            print("calling button:", button.text())
            button.setEnabled(False)
            func(*args, **kwargs)
            button.setEnabled(True)
            print("done:", button.text())

        worker = Worker(wrapped)
        self.pool.start(worker)

    def action(self, name):
        print(f"[ACTION] {name}")

def main():
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # allow Ctrl +C to exit

    app = QtWidgets.QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
