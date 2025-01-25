import tkinter as tk
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from ament_index_python.packages import get_package_share_directory
import os


class ROS2Node(Node):
    """ROS2 Node for publishing messages on button clicks."""

    def __init__(self):
        super().__init__("gui_publisher_node")
        self.publisher_LF_grieel_command = self.create_publisher(String, "/LF/lbr_low_level_controller/grieel_command", 10)
        self.publisher_LH_grieel_command = self.create_publisher(String, "/LH/lbr_low_level_controller/grieel_command", 10)
        self.publisher_RH_grieel_command = self.create_publisher(String, "/RH/lbr_low_level_controller/grieel_command", 10)
        self.publisher_RF_grieel_command = self.create_publisher(String, "/RF/lbr_low_level_controller/grieel_command", 10)

    def publish_message(self, message):
        """Publish a message to the topic."""
        msg_parts = message.split()
        msg = String()
        msg.data = msg_parts[1]
        if msg_parts[0] == 'LF':
            self.publisher_LF_grieel_command.publish(msg)
        if msg_parts[0] == 'LH':
            self.publisher_LH_grieel_command.publish(msg)
        if msg_parts[0] == 'RH':
            self.publisher_RH_grieel_command.publish(msg)
        if msg_parts[0] == 'RF':
            self.publisher_RF_grieel_command.publish(msg)

        self.get_logger().info(f"Published message: {message}")


class GUIApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS2 GUI Example")
        
        # Set up ROS 2 node in a separate thread
        rclpy.init()
        self.ros_node = ROS2Node()
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()
        
        # Set up GUI layout
        self.left_frame = tk.Frame(root, width=1400,height=1275, bg="#C8C8C8")
        self.left_frame.pack(side="left", fill="both", expand=True)
        
        self.right_frame = tk.Frame(root, width=600, height=1275, bg="white")
        self.right_frame.pack(side="right", fill="both", expand=True)
        
        # Canvas for background image
        self.canvas = tk.Canvas(self.left_frame, bg="#C8C8C8", width=1400, height=1275)
        self.canvas.pack(fill="both", expand=True)
        
        # Load background image
        package_path = os.path.join(get_package_share_directory('grieel_gui'), 'resources')
        self.image_paths = [package_path + "/LIMBERO.png"]  # Add paths to your images
        self.current_image_index = 0
        self.load_background_image(self.image_paths[self.current_image_index])
        
        # Add image buttons
        self.buttons_data = [
            {"image_path": os.path.join(package_path, "WheelLF.png"), "message": "LF wheel", "x": 90, "y": 230},
            {"image_path": os.path.join(package_path, "GripperLF.png"), "message": "LF gripper", "x": 260, "y": 60},
            {"image_path": os.path.join(package_path, "WheelLH.png"), "message": "LH wheel", "x":90 , "y":900 },
            {"image_path": os.path.join(package_path, "GripperLH.png"), "message": "LH gripper", "x": 260, "y": 1070},
            {"image_path": os.path.join(package_path, "WheelRH.png"), "message": "RH wheel", "x":1040 , "y": 900 },
            {"image_path": os.path.join(package_path, "GripperRH.png"), "message": "RH gripper", "x": 870, "y": 1070},
            {"image_path": os.path.join(package_path, "WheelRF.png"), "message": "RF wheel", "x": 1040, "y": 230},
            {"image_path": os.path.join(package_path, "GripperRF.png"), "message": "RF gripper", "x": 870, "y": 60},
        ]
        self.buttons = []  # To store buttons

        for btn_data in self.buttons_data:
            button_image = Image.open(btn_data["image_path"])
            button_image = button_image.resize((150, 150), Image.Resampling.LANCZOS)  # Resize as needed
            button_image_tk = ImageTk.PhotoImage(button_image)

            button = tk.Button(
                self.canvas,
                image=button_image_tk,
                command=lambda msg=btn_data["message"]: self.on_button_click(msg),
                bd=0,  # Borderless button
            )
            button.image = button_image_tk  # Keep a reference to avoid garbage collection
      
            self.buttons.append(button)
            # Place the button on the canvas
            self.canvas.create_window(btn_data["x"], btn_data["y"], anchor="nw", window=button)
        
        # Right frame placeholder for information display
        self.info_label = tk.Label(self.right_frame, text="Information", bg="white", font=("Arial", 14))
        self.info_label.pack(pady=20)

    def load_background_image(self, image_path):
        """Load and display the background image on the canvas."""
        self.background_image = Image.open(image_path)
        self.background_image = self.background_image.resize((1280, 1275), Image.Resampling.LANCZOS)
        self.background_image_tk = ImageTk.PhotoImage(self.background_image)
        self.canvas.create_image(0, 0, anchor="nw", image=self.background_image_tk)

    def on_button_click(self, message):
        """Handle button click events."""
        # Update information label
        self.info_label.config(text=f"Published: {message}")
        # Publish the message using ROS 2
        self.ros_node.publish_message(message)


def main():
    # Create the main window
    root = tk.Tk()
    app = GUIApp(root)
    root.geometry("2000x1275")  # Set window size

    try:
        root.mainloop()
    finally:
        # Shut down ROS 2 when GUI is closed
        rclpy.shutdown()


if __name__ == "__main__":
    main()
