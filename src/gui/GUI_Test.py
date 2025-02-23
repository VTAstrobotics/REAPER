import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk  # Required for PNG handling
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MultiTopicSubscriber(Node):

    def __init__(self):

        super().__init__("multi_topic_subscriber")
        self.custom_subscriptions = {}  
        self.messages = {}

    def subscribe_to_topic(self, topic_name):

        if topic_name in self.custom_subscriptions:

            self.get_logger().warn(f"Already subscribed to topic: {topic_name}")
            return

        def callback(msg, topic=topic_name):

            self.messages[topic] = msg.data

        subscription = self.create_subscription(

            String, topic_name, callback, 10

        )

        self.custom_subscriptions[topic_name] = subscription
        self.messages[topic_name] = "No data received yet"
        self.get_logger().info(f"Subscribed to topic: {topic_name}")
 
class TkMultiTopicApp:
    
    def __init__(self, root, ros_node):

        # Initial root and node
        self.root = root
        self.ros_node = ros_node

        # Set colors
        self.root.configure(bg="white")

        # Frame 
        self.input_frame = tk.Frame(root, bg="white")
        self.input_frame.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        # Normal topic labels, grid, and buttons
        self.topic_label = tk.Label(self.input_frame, text="Enter Topic Name:", bg="white", fg="red")
        self.topic_label.grid(row=0, column=0, padx=5, pady=5)

        self.topic_entry = tk.Entry(self.input_frame, width=30)
        self.topic_entry.grid(row=0, column=1, padx=5, pady=5)

        self.subscribe_button = tk.Button(
            self.input_frame, text="Subscribe", command=self.subscribe_to_topic, bg="red", fg="white"
        )
        self.subscribe_button.grid(row=0, column=2, padx=5, pady=5)

        # Camera topic labels, grid, and buttons
        self.camera_label = tk.Label(self.input_frame, text="Enter Camera Name:", bg="white", fg="red")
        self.camera_label.grid(row=1, column=0, padx=5, pady=5)

        self.camera_entry = tk.Entry(self.input_frame, width=30)
        self.camera_entry.grid(row=1, column=1, padx=5, pady=5)

        self.camera_subscribe_button = tk.Button(
            self.input_frame, text="Subscribe", command=self.subscribe_to_camera_topic, bg="red", fg="white"
        )
        self.camera_subscribe_button.grid(row=1, column=2, padx=5, pady=5)

        # Frame for messages
        self.messages_frame = tk.Frame(root, bg="red")
        self.messages_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # Configure the root grid layout
        root.grid_rowconfigure(1, weight=1)
        root.grid_columnconfigure(0, weight=1)

        self.messages_widgets = {}  
        self.dragging_widget = None 
        self.offset_x = 0  
        self.offset_y = 0 
        self.is_dragging = False   

        # Start a thread to update the messages
        self.update_message_thread = threading.Thread(target=self.update_messages)
        self.update_message_thread.daemon = True
        self.update_message_thread.start()

        # Launch RViz in a subprocess
        self.launch_rviz()

    # Subscription logic 
    def subscribe_to_topic(self):

        topic_name = self.topic_entry.get().strip()

        if topic_name:

            if topic_name in self.messages_widgets:

                messagebox.showinfo("Already Subscribed", f"Already subscribed to {topic_name}.")
                return

            try:

                self.ros_node.subscribe_to_topic(topic_name)
                self.add_topic_label(topic_name)

            except Exception as e:

                messagebox.showerror("Error", f"Failed to subscribe: {e}")
        else:

            messagebox.showerror("Invalid Input", "Please enter a valid topic name.")

    def subscribe_to_camera_topic(self):

        camera_topic_name = self.camera_entry.get().strip()

        if camera_topic_name:

            if camera_topic_name in self.messages_widgets:

                messagebox.showinfo("Already Subscribed", f"Already subscribed to {camera_topic_name}.")
                return

            try:

                self.ros_node.subscribe_to_topic(camera_topic_name)
                self.add_camera_label(camera_topic_name)

            except Exception as e:

                messagebox.showerror("Error", f"Failed to subscribe: {e}")

        else:

            messagebox.showerror("Invalid Input", "Please enter a valid camera topic name.")

    # Labeling logic
    def add_topic_label(self, topic_name):
        
        label_frame = tk.Frame(self.messages_frame, bg="white", bd=1, relief="solid")
        label_frame.place(x=10, y=10)  
        label_topic = tk.Label(label_frame, text=f"Topic: {topic_name}", font=("Arial", 10, "bold"))
        label_topic.pack(side="top", padx=5, pady=2)
        label_message = tk.Label(label_frame, text="No data received yet", font=("Arial", 12))
        label_message.pack(side="top", padx=5, pady=2)

        # Enable dragging 
        self.bind_drag_events(label_frame)

        # Right-click menu 
        self.create_right_click_menu(label_frame, topic_name)

        self.messages_widgets[topic_name] = {
            "frame": label_frame,
            "message_label": label_message,
            "last_data": None,
            "timeout_timer": None
        }

    def add_camera_label(self, camera_topic_name):
 
        label_frame = tk.Frame(self.messages_frame, bg="lightblue", bd=1, relief="solid")
        label_frame.place(x=10, y=10)  # Default position
        label_camera = tk.Label(label_frame, text=f"Camera: {camera_topic_name}", font=("Arial", 10, "bold"))
        label_camera.pack(side="top", padx=5, pady=2)
        label_message = tk.Label(label_frame, text="No data received yet", font=("Arial", 12))
        label_message.pack(side="top", padx=5, pady=2)

        # Enable dragging 
        self.bind_drag_events(label_frame)

        # Right-click menu 
        self.create_right_click_menu(label_frame, camera_topic_name)

        self.messages_widgets[camera_topic_name] = {
            "frame": label_frame,
            "message_label": label_message,
            "last_data": None,
            "timeout_timer": None
        }

    def create_right_click_menu(self, frame, topic_name):
 
        def remove_topic():
            self.remove_topic(topic_name)

        right_click_menu = tk.Menu(self.root, tearoff=0)
        right_click_menu.add_command(label="Remove Topic", command=remove_topic)

        # Bind the right-click event to show the menu
        frame.bind("<Button-3>", lambda event, menu=right_click_menu: menu.post(event.x_root, event.y_root))

        # Bind the same right-click event to the child labels to trigger the context menu
        for child in frame.winfo_children():
            child.bind("<Button-3>", lambda event, menu=right_click_menu: menu.post(event.x_root, event.y_root))

    def remove_topic(self, topic_name):
 
        if topic_name in self.messages_widgets:
            
            # Cancel the timeout timer if it's running
            if self.messages_widgets[topic_name]["timeout_timer"]:
                self.messages_widgets[topic_name]["timeout_timer"].cancel()

            # Destroy the frame associated with the topic
            self.messages_widgets[topic_name]["frame"].destroy()
            del self.messages_widgets[topic_name]

    # Dragging logic
    def bind_drag_events(self, widget):

        widget.bind("<Button-1>", self.start_drag)
        widget.bind("<B1-Motion>", self.do_drag)
        widget.bind("<ButtonRelease-1>", self.stop_drag)

        for child in widget.winfo_children():
            self.bind_drag_events(child)

    def start_drag(self, event):

        self.dragging_widget = event.widget

        while not isinstance(self.dragging_widget, tk.Frame):
            self.dragging_widget = self.dragging_widget.master

        self.offset_x = event.x  
        self.offset_y = event.y 
        self.is_dragging = True

    def do_drag(self, event):

        if self.dragging_widget and self.is_dragging:
            # Calculate new position 
            x = self.dragging_widget.winfo_x() + event.x - self.offset_x
            y = self.dragging_widget.winfo_y() + event.y - self.offset_y
            # Update position
            self.dragging_widget.place(x=x, y=y)

    def stop_drag(self, event):

        self.is_dragging = False
        self.dragging_widget = None

    # Show messgae for no data or stale data
    def update_messages(self):

        while rclpy.ok():

            rclpy.spin_once(self.ros_node, timeout_sec=0.1)

            for topic_name, widgets in self.messages_widgets.items():

                new_message = self.ros_node.messages.get(topic_name, "No data received yet")
                label_message = widgets["message_label"]

                if new_message != widgets["last_data"]:  

                    label_message.config(text=new_message)
                    widgets["last_data"] = new_message

                    # Restart timeout timer
                    if widgets["timeout_timer"]:
                        widgets["timeout_timer"].cancel()

                    self.check_data_timeout(topic_name, label_message)

    def check_data_timeout(self, topic_name, label_message):
 
        def timeout_action():
            if label_message.cget("text") != "No data received yet":
                label_message.config(text="Data timed out")

        timer = threading.Timer(3, timeout_action)
        self.messages_widgets[topic_name]["timeout_timer"] = timer
        timer.start()

    # Launches rviz
    def launch_rviz(self):
        rviz_cmd = ["ros2", "run", "rviz2", "rviz2"]
        # try:
            # subprocess.Popen(rviz_cmd)
        # except Exception as e:
            # messagebox.showerror("Error", f"Failed to launch RViz: {e}")

def main():

    rclpy.init()
    ros_node = MultiTopicSubscriber()
 
    root = tk.Tk()
    root.title("Andrew GUI Test")
    root.geometry("800x600")  
    app = TkMultiTopicApp(root, ros_node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
