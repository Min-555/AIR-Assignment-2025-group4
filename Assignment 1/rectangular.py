#!/usr/bin/env python3
import threading #多线程控制
import time #时间控制
import rclpy #ROS2的python库（创建节点、发布订阅等））
from rclpy.node import Node # ROS2节点的基类
from rclpy import Parameter # ROS2的参数库，用于创建和管理参数
from interfaces.msg import QuadControlTarget # QuadControlTarget消息类型，用于控制四足机器人
from interfaces.msg import QuadState # QuadState消息类型，用于获取四足机器人的状态
from rcl_interfaces.srv import SetParameters # 服务接口，用于动态设置参数
from tf_transformations import euler_from_quaternion


# The following interfaces are available from the simulator and controller to interact with the robot:
# QuadState (from simulator):
#     std_msgs/Header header
#     geometry_msgs/PoseWithCovariance pose
#     geometry_msgs/TwistWithCovariance twist
#     geometry_msgs/Accel acceleration
#     JointState joint_state
#     bool[4] foot_contact
#     float64[12] ground_contact_force # 4 times xyz
#     bool belly_contact
# QuadControlTarget (sent to controller):
#     # linear velocities
#     float64 body_x_dot
#     float64 body_y_dot
#     # height
#     float64 world_z
#     # angular velocity
#     float64 hybrid_theta_dot
#     # orientation
#     float64 pitch
#     float64 roll


# 定义一个四足机器人行走控制器类
# WalkingController类继承自Node类

class WalkingController(Node): 
    def __init__(self):
        super().__init__('walking_square_controller') # 初始化节点名称为walking_controller
        self.shutdown_flag = False # 初始化关闭标志为False
        self.current_step = 0 #用於跟蹤矩形路勁的當前步驟
        # Basic Interfaces to the Simulator and Controller
        # Get State from the Simulator
        # Create a subscription to receive the state of the quadruped robot
        self.quad_state_sub = self.create_subscription(QuadState, 'quad_state', self.quad_state_callback, 1) # 订阅四足机器人状态消息，队列大小为1
        # Print Callback function for simple debugging
        self.print_timer = self.create_timer(2.0, self.print_callback) # 定时器函数，每2秒调用一次print_callback函数，用于调试和查看输出
        # Create a publisher to send commands to the controller
        self.quad_control_target_pub = self.create_publisher(QuadControlTarget, 'quad_control_target', 10) # 发布四足机器人控制目标消息，队列大小为10
        self.quad_state = None # Initialize quad_state为None，表示四足机器人状态未初始化

        # self.step_timer = self.create_timer(0.1, self.step_callback) # 定时器函数，每0.1秒调用一次step_callback函数

        # Timer Function for logging at 10 Hz/0.1s
        self.log_timer = self.create_timer(0.1, self.log_callback) # 定时器函数，每0.1秒调用一次log_callback函数，用于记录日志
        # Log Limit: Not used at the moment.
        self.log_limit = 10 # 日志限制，当前未使用
        # Log File
        self.log_file = "log.txt" # 日志文件名
        self.log_header = False # 日志头部标志，初始为False，表示未写入头部
        # Clear Log file at start: WARNING: THIS WILL CLEAR LAST EXPERIMENT DATA!!
        with open(self.log_file, "w") as log_file:
            print("WARNING: Log file cleared!")

        # Command the Robot to Move
        self.param_client = self.create_client(SetParameters, '/mit_controller_node/set_parameters') # 创建一个客户端，用于设置参数，服务名称为/mit_controller_node/set_parameters

    # Receive the quad state from the simulator
    def quad_state_callback(self, msg):
        self.quad_state = msg

    # Callback function for the quad state, used for debugging and viewing outputs
    def print_callback(self):
        # Function for debugging and viewing outputs
        if self.quad_state is not None:
            # print(f"Quad state: {self.quad_state}")
            print(f"Position: {self.quad_state.pose.pose.position}")
            print(f"X Velocity: {self.quad_state.twist.twist.linear.x}")
            print(f"Y Velocity: {self.quad_state.twist.twist.linear.y}")
            print(f"orientation: {self.quad_state.pose.pose.orientation}")
            print(f"Step: {self.current_step}")
            pass

    def set_gait(self, gait):
        param_req = SetParameters.Request() # 创建一个SetParameters请求对象
        param_req.parameters = [
                Parameter(name='simple_gait_sequencer.gait', value=gait).to_parameter_msg()
        ]
        self.param_client.call_async(param_req) # 设置参数请求
        print(f"Set gait to {gait}") # 打印设置的步态
        time.sleep(1.0)
        
    def publish_command(self, x_dot, y_dot=0.0, world_z=0.3, theta_dot=0.0, pitch=0.0, roll=0.0):
        cmdMsg = QuadControlTarget()
        cmdMsg.body_x_dot = x_dot # 设置x方向速度
        cmdMsg.body_y_dot = y_dot
        cmdMsg.world_z = world_z # 设置高度
        cmdMsg.hybrid_theta_dot = theta_dot # 设置角速度
        cmdMsg.pitch = pitch
        cmdMsg.roll = roll # 设置滚转角
        self.quad_control_target_pub.publish(cmdMsg) # 发布控制目标消息

        
    def move_in_square(self):
        while self.quad_state is None: # Wait for the quad_state to be initialized
            self.get_logger().info("Waiting for quad_state...")
            time.sleep(0.1)
        
        if self.quad_state is not None: 
            self.get_logger().info("Moving forward.")
            self.set_gait("WALKING_TROT")
            
            while self.current_step == 0: # Move 2 meter along the x axis
                self.publish_command(x_dot=0.2)
                if self.quad_state.pose.pose.position.x >= 2.0:
                    self.publish_command(0.0)
                    self.current_step = 1
                    print("Step 1 complete: Moving along x axis to 2.0m")
                    break
                
            while self.current_step == 1: # Turn left 90 degrees
                self.publish_command(0.0, theta_dot=0.2)
                if self.quad_state.pose.pose.orientation.z >= 1.57:
                    self.publish_command(0.0)
                    self.current_step = 2
                    print("Step 2 complete: Turning left 90 degrees")
                    break
            while self.current_step == 2: # Move 2 meter along the y axis
                self.publish_command(x_dot=0.2)
                if self.quad_state.pose.pose.position.y >= 2.0:
                    self.publish_command(0.0)
                    self.current_step = 3
                    print("Step 3 complete: Moving along y axis to 2.0m")
                    break
            while self.current_step == 3: # Turn left 90 degrees
                self.set_gait("STAND") # Set the gait to STAND
                self.shutdown_flag = True # Set the shutdown flag to True
                
                
            # if self.current_step == 0: # Move 2 meter along the x axis
            #     if self.quad_state.pose.pose.position.x < 2.0:
            #         self.publish_command(x_dot=0.2)
            #         print(f"Robot is starting to walk forward.")
            #     else:
            #         self.current_step = 1
            #         self.publish_command(0.0)
            #         print("Step 1 complete: Moving along x axis to 2.0m")
            # elif self.current_step == 1: # Turn left 90 degrees
            #     if self.quad_state.pose.pose.angle.z < np.pi/2:
            #         self.publish_command(0.0, theta_dot=0.2)
            #     else: 
            #         self.current_step = 2
            #         print("Step 2 complete: Turning left 90 degrees")
            # elif self.current_step == 2: # Move 2 meter along the y axis
            #     if self.quad_state.pose.pose.position.y < 2.0:
            #         self.publish_command(0.2)
            #     else:
            #         self.current_step = 3
            #         print("Step 3 complete: Moving along y axis to 2.0m")
            # elif self.current_step == 3:
            #     print("Step 4 complete: Finished")
            #     self.publish_command(0.0) # Stop the robot
            #     self.set_gait("STAND") # Set the gait to STAND
            #     self.shutdown_flag = True # Set the shutdown flag to True
            

    def log_callback(self):
        if self.quad_state is not None:
            if self.log_header:
                # Once we have the header, we can write the data
                with open(self.log_file, "a") as log_file:
                    log_file.write(f"{self.quad_state.pose.pose.position.x}, {self.quad_state.pose.pose.position.y}, {self.quad_state.pose.pose.position.z}, {self.quad_state.twist.twist.linear.x}, {self.quad_state.twist.twist.linear.y}, {self.quad_state.twist.twist.linear.z}, {self.quad_state.twist.twist.angular.x}, {self.quad_state.twist.twist.angular.y}, {self.quad_state.twist.twist.angular.z}, \n")
            else:
                # Make the header first
                with open(self.log_file, "a") as log_file:
                    log_file.write("x_pos, y_pos, z_pos, x_vel, y_vel, z_vel,  x_ang_vel, y_ang_vel, z_ang_vel, \n")
                self.log_header = True


def main(args=None):
    rclpy.init(args=args) # 初始化ROS2
    walking_controller = WalkingController() # 创建一个WalkingController对象
    # Create a thread to run the executor
    # This will allow the node to run in a separate thread and avoid blocking the main thread
    def run_func():
        executor = rclpy.executors.MultiThreadedExecutor(1) # 创建一个多线程执行器，队列大小为1
        executor.add_node(walking_controller) # 将walking_controller节点添加到执行器中
        while rclpy.ok():
            walking_controller.get_logger().info("Running") # 打印日志，表示节点正在运行
            executor.spin() # 执行器运行
            return

    runner = threading.Thread(target=run_func) # 创建一个线程，目标函数为run_func
    runner.start() # 启动线程
    
    walking_controller.move_in_square()
    # Wait for the thread to finish
    while rclpy.ok(): 
        if walking_controller.shutdown_flag:
            break
        time.sleep(0.1)
        
    walking_controller.destroy_node() # 销毁节点
    rclpy.shutdown() # 关闭ROS2节点

if __name__ == "__main__":
    main()
