import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import os
import numpy
import csv 

JOINT_NUM = 7
class DataAnalysis(Node):

    def __init__(self):
        super().__init__('lbr_data_analysis')
        self.get_logger().info(self.get_name() + " node start")

        self.joint_states_ = JointState()
        self.joint_reference_ = JointState()
        self.joint_reference_.name = [  "LF_B2C", "LF_C2F", "LF_F2T", "LF_T2E", "LF_wristH", "LF_wristV", "LF_driving",
                                        "LH_B2C", "LH_C2F", "LH_F2T", "LH_T2E", "LH_wristH", "LH_wristV", "LH_driving",
                                        "RH_B2C", "RH_C2F", "RH_F2T", "RH_T2E", "RH_wristH", "RH_wristV", "RH_driving",
                                        "RF_B2C", "RF_C2F", "RF_F2T", "RF_T2E", "RF_wristH", "RF_wristV", "RF_driving"]
        self.joint_states_.name = self.joint_reference_.name

        self.joint_reference_.position = numpy.zeros(len(self.joint_reference_.name)).tolist()
        self.joint_states_.position = numpy.zeros(len(self.joint_states_.name)).tolist()

        self.file_name = 'lbr_data.csv'
        self.file_path = '/home/ale/lbr_ws/src/LIMBERO/TOOLS/lbr_data_analysis/script/'+self.file_name

        self.file = open(self.file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.file)
        
        # Write header
        self.csv_writer.writerow(['time', 'joint_name', 'joint_state', 'joint_reference'])

        self.joint_states_subscriber_ = self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.joint_reference_subscriber_LF_ = self.create_subscription(JointState, "/LF/lbr_limb_controller/joint_state", self.joint_reference_callback_LF, 10)
        self.joint_reference_subscriber_LH_ = self.create_subscription(JointState, "/LH/lbr_limb_controller/joint_state", self.joint_reference_callback_LH, 10)
        self.joint_reference_subscriber_RH_ = self.create_subscription(JointState, "/RH/lbr_limb_controller/joint_state", self.joint_reference_callback_RH, 10)
        self.joint_reference_subscriber_RF_ = self.create_subscription(JointState, "/RF/lbr_limb_controller/joint_state", self.joint_reference_callback_RF, 10)
    


    def joint_states_callback(self, msg):
        for name in self.joint_states_.name:
            joint_index_msg = msg.name.index(name)
            self.joint_states_.position[self.joint_states_.name.index(name)] = msg.position[joint_index_msg]

        if self.joint_reference_ is not None:
            time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec / 1e9
            name = ', '.join(map(str,self.joint_states_.name))
            positions = ', '.join(map(str, self.joint_states_.position))
            control_positions = ','.join(map(str,self.joint_reference_.position))

            self.csv_writer.writerow([time, name, positions, control_positions])


    def joint_reference_callback_LF(self,msg):
        limb_id = 0
        self.joint_reference_.position[limb_id*JOINT_NUM:limb_id*JOINT_NUM+7] = msg.position
    def joint_reference_callback_LH(self,msg):
        limb_id = 1
        self.joint_reference_.position[limb_id*JOINT_NUM:limb_id*JOINT_NUM+7] = msg.position
    def joint_reference_callback_RH(self,msg):
        limb_id = 2
        self.joint_reference_.position[limb_id*JOINT_NUM:limb_id*JOINT_NUM+7] = msg.position
    def joint_reference_callback_RF(self,msg):
        limb_id = 3
        self.joint_reference_.position[limb_id*JOINT_NUM:limb_id*JOINT_NUM+7] = msg.position

        


def main(args=None):
    rclpy.init(args=args)

    lbr_data_analysis_node = DataAnalysis()
    rclpy.spin(lbr_data_analysis_node)
    rclpy.shutdown()
    lbr_data_analysis_node.file.close()


if __name__ == '__main__':
    main()
