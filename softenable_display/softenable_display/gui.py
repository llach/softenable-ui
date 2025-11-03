import os
import sys
import time
import rclpy
import threading
import subprocess

from PyQt5 import QtWidgets, uic  # or PyQt6 if you prefer
from PyQt5.QtCore import QThreadPool, QRunnable, pyqtSlot
from ament_index_python.packages import get_package_share_directory

from rclpy.node import Node
from stack_msgs.srv import RollerGripper
from softenable_display_msgs.srv import SetDisplay


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
        self.cli_display = self.create_client(SetDisplay, "set_display")
        self.gripper_l_cli = self.create_client(RollerGripper, "left_roller_gripper")
        self.gripper_r_cli = self.create_client(RollerGripper, "right_roller_gripper")

        ### check if services are running. not good when debugging.
        # for cli in [
        #     self.cli_display,
        #     self.gripper_l_cli,
        #     self.gripper_r_cli
        # ]:
        #     if not cli.wait_for_service(timeout_sec=10.0):
        #         self.get_logger().warn(f"Service '{cli.service_name}' not available")
        #         exit(-1)

    def open_grippers(self):
        print("opening gripper")

        fut_l = self.gripper_l_cli.call_async(RollerGripper.Request(finger_pos=2000))
        fut_r = self.gripper_r_cli.call_async(RollerGripper.Request(finger_pos=2500))

        self.wait_for_futures([fut_l, fut_r])

    def close_grippers(self):
        print("closing gripper")

        fut_l = self.gripper_l_cli.call_async(RollerGripper.Request(finger_pos=3400))
        fut_r = self.gripper_r_cli.call_async(RollerGripper.Request(finger_pos=800))

        self.wait_for_futures([fut_l, fut_r])

    def set_display(self, preset):
        print(f"setting display preset {preset}")

        self.wait_for_futures([
            self.cli_display.call_async(SetDisplay.Request(name=preset))
        ])

    def wait_for_futures(self, futures):
        for f in futures: 
            while not f.done() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)

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
        self.btnBagInitial.clicked.connect(lambda: self.run_with_disable(self.btnBagInitial, self.ros2_run, "stack_approach", "bag_opening", args="initial"))
        self.btnBagInitialNew.clicked.connect(lambda: self.run_with_disable(self.btnBagInitialNew, self.ros2_run, "stack_approach", "bag_opening", args="initial_new"))
        self.btnBagDemo.clicked.connect(lambda: self.run_with_disable(self.btnBagDemo, self.ros2_run, "stack_approach", "bag_opening_perc", args="slides"))
        self.btnBagOpen.clicked.connect(lambda: self.run_with_disable(self.btnBagOpen, self.open_and_slide))
        self.btnBagRetreat.clicked.connect(lambda: self.run_with_disable(self.btnBagRetreat, self.ros2_run, "stack_approach", "bag_opening", args="retreat"))

        self.btnUnstackDemo.clicked.connect(lambda: self.run_with_disable(self.btnUnstackDemo, self.ros2_run, "softenable_bt", "grasp_first_layer", modify_ld=True))
        self.btnUnstackSlideEight.clicked.connect(lambda: self.run_with_disable(self.btnUnstackSlideEight, self.slide_eight))
        self.btnUnstackSlides.clicked.connect(lambda: self.run_with_disable(self.btnUnstackSlides, self.final_slides))

        self.btnUnfold.clicked.connect(lambda: self.run_with_disable(self.btnUnstackDemo, self.python_ros2_run, "src/gown_opening/gown_opening/dual_wo_yolo.py"))

        self.btnToolsOpen.clicked.connect(lambda: self.run_with_disable(self.btnToolsOpen, self.node.open_grippers))
        self.btnToolsClose.clicked.connect(lambda: self.run_with_disable(self.btnToolsClose, self.node.close_grippers))

    def run_with_disable(self, button, func, *args, **kwargs):

        def wrapped():
            print("calling button:", button.text())
            button.setEnabled(False)
            func(*args, **kwargs)
            button.setEnabled(True)
            print("done:", button.text())

        worker = Worker(wrapped)
        self.pool.start(worker)

    def open_and_slide(self):
        self.node.open_grippers()
        # self.node.set_display("protocol_bag_3")

    def slide_eight(self):
        self.node.set_display("protocol_8")

    def final_slides(self):
        for p in [
            "protocol_9",
            "protocol_10"
        ]:
            self.node.set_display(p)
            time.sleep(5)

    def ros2_run(self, package, executable, blocking=True, args="", modify_ld=False):
        print(f"ROS2 running '{executable}' from package '{package}'")

        # proc = subprocess.Popen(["ros2", "run", package, executable])
        my_env = os.environ.copy()
        if modify_ld:
            my_env['LD_LIBRARY_PATH'] = f"/opt/conda/envs/softenable/lib/:{my_env['LD_LIBRARY_PATH']}"

        proc = subprocess.Popen(f"ros2 run {package} {executable} {args}".split(" "), env=my_env)
        if blocking:
            print("waiting for process to finish ...")
            proc.wait()
        print("all done!")

    def python_ros2_run(self, executable, blocking=True, args=""):
        print(f"ROS2 running '{executable}'")

        # proc = subprocess.Popen(["ros2", "run", package, executable])
        proc = subprocess.Popen(f"LD_LIBRARY_PATH=/opt/conda/envs/softenable/lib/:$LD_LIBRARY_PATH  python {executable} {args}".split(" "))
        if blocking:
            print("waiting for process to finish ...")
            proc.wait()
        print("all done!")

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
