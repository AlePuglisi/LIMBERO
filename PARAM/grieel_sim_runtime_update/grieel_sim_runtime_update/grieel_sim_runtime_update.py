import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelList, DeleteEntity, SpawnEntity
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import xacro
import os
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController

class GrieelSimRuntimeUpdate(Node):
    def __init__(self):
        super().__init__('grieel_sim_runtime_update')

        # Service clients
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Wait for services to be available
        self.wait_for_service(self.delete_entity_client, '/delete_entity')
        self.wait_for_service(self.spawn_entity_client, '/spawn_entity')
        self.wait_for_service(self.switch_controller_client, '/controller_manager/switch_controller')

        # Variables to store joint states
        self.joint_states = None

        self.timer_spawn = None
        self.timer_delete = None
        self.timer_controller = None

        self.trigger_subscriber_ = self.create_subscription(
            String, 'grieel_mode_change', self.grieel_update_callback, 10)
        self.joint_subscriber_ = self.create_subscription(
            JointState, 'lbr_state_estimator/joint_state', self.update_joint_state, 10)
        self.joint_publisher_ = self.create_publisher(JointState,'grieel_sim_runtime_update/joint_state', 10)
        

    def wait_for_service(self, client, service_name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting...')

    def update_joint_state(self, msg):
        self.joint_states = msg

    def grieel_update_callback(self, msg):
        self.get_logger().info('Joint states saved')
        self.delete_model('LIMBERO')

    def delete_model(self, model_name):
        self.get_logger().info(f'Deleting model {model_name}...')
        req = DeleteEntity.Request()
        req.name = model_name
        future = self.delete_entity_client.call_async(req)
        self.timer_delete = self.create_timer(1.0, lambda: self.check_future(future, self.handle_delete_model))
    
    def handle_delete_model(self, future):
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Model deleted successfully')
                self.spawn_model_with_new_urdf('LIMBERO')  
                self.destroy_timer(self.timer_delete)
            else:
                self.get_logger().error('Failed to delete model, success flag is False')
        else:
            self.get_logger().error('DeleteEntity service call failed')

    def spawn_model_with_new_urdf(self, model_name):
        urdf_file_path = os.path.join(get_package_share_directory('lbr_description'), 'urdf', 'Grieel/LIMBERO_GRIEEL.xacro') 

        # Process the xacro file with new arguments
        names = []
        for name in self.joint_states.name:
            names.append(name + "_initial_value")

        joints = []
        for joint in self.joint_states.position:
            joints.append(str(joint))

        initial_joints = dict(zip(names,joints))  
        wheel_modes = {'LF_wheel_mode': 'false', "LH_wheel_mode": 'false','RH_wheel_mode': 'false', "RF_wheel_mode": 'true'}
        args = initial_joints | wheel_modes
        
        arg_list = []
        for key,value in args.items():
            arg_list.append(f"{key}:={value}")

        urdf_xml = xacro.process_file(urdf_file_path, mappings=args).toxml()

        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.4
        initial_pose.orientation.x = 0.0
        initial_pose.orientation.y = 0.0
        initial_pose.orientation.z = 0.0
        initial_pose.orientation.w = 1.0

        # Create and configure the spawn entity request
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = urdf_xml
        req.initial_pose = initial_pose
        req.robot_namespace = '/'
        req.reference_frame = 'world'

        future = self.spawn_entity_client.call_async(req)
        self.timer_spawn = self.create_timer(1.0, lambda: self.check_future(future, self.handle_spawn_model))

    def switch_controllers(self, stop_controllers, start_controllers):
        req = SwitchController.Request()
        req.stop_controllers = stop_controllers
        req.start_controllers = start_controllers

        future = self.switch_controller_client.call_async(req)
        self.get_logger().info("respawning controllers 1") #debug
        self.timer_controller = self.create_timer(1.0, lambda: self.check_future(future, self.handle_controller_respawn))
    
    def handle_controller_respawn(self,future):
            self.get_logger().info("respawning controllers 2") #debug
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'controllers spawned successfully')
                    self.destroy_timer(self.timer_controller)
                else:
                    self.get_logger().error('Failed to respawn controllers, success flag is False')
            else:
                self.get_logger().error('switch controllers service call failed')

    
    def handle_spawn_model(self, future):
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Model spawned successfully')
                self.set_joint_states()
                stop_controllers = ['joint_state_broadcaster', 'joint_trajectory_controller']
                start_controllers = ['joint_state_broadcaster', 'joint_trajectory_controller']
                self.switch_controllers(stop_controllers, start_controllers)
                self.destroy_timer(self.timer_spawn)
            else:
                self.get_logger().error('Failed to spawn model, success flag is False')
        else:
            self.get_logger().error('SpawnEntity service call failed')

    def set_joint_states(self):
        if self.joint_states is not None:
            self.joint_publisher_.publish(self.joint_states)
            self.get_logger().info('Setting joint states for the new model...')
            print("setted joints", self.joint_states)
        else:
            self.get_logger().error('No joint states saved to set')

    def check_future(self, future, callback):
        if future.done():
            self.get_logger().info("Future is done")  # debug
            try:
                result = future.result()
                self.get_logger().info(f"Future result: {result}")
                callback(future)
            except Exception as e:
                self.get_logger().error(f"Exception in future: {e}")
                callback(future)

def main(args=None):
    rclpy.init(args=args)
    robot_manager = GrieelSimRuntimeUpdate()
    rclpy.spin(robot_manager)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    