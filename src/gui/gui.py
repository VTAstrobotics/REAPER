import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as RosImage
import cv2
from cv_bridge import CvBridge
import time
from functools import partial

# Test Talker Node
# ros2 run demo_nodes_cpp talker
# ros2 run camera_streamer usbCamStreamer --cam 0

# Joystick topics
joystick_topic_1 = "/joy"
joystick_topic_2 = "/operator/joy"
 
class MultiTopicSubscriber(Node):

    def __init__(self):

        super().__init__("multi_topic_subscriber")
        self.custom_subscriptions = {}  
        self.messages = {}
        self.bridge = CvBridge()
        self.camera_frames = {}

        # ——— joystick “last seen” tracking ———
        import time
        self.joystick_last_seen = {
            joystick_topic_1: 0.0,
            joystick_topic_2: 0.0
        }
        self.joystick_subscriptions = {}

        # subscribe once to each, to update last-seen timestamp
        for topic in (joystick_topic_1, joystick_topic_2):
            try:
                sub = self.create_subscription(
                    Joy, topic,
                    partial(self._joystick_callback, topic_name=topic),
                    #lambda msg, t=topic: self._joystick_callback(msg, t),
                    10)
                self.joystick_subscriptions[topic] = sub
                self.get_logger().info(f"Listening for joystick on {topic}")
            except Exception:
                # if topic doesn't exist yet, we'll still create it once it appears
                 pass

    def _joystick_callback(self, msg, topic_name: str):
        self.joystick_last_seen[topic_name] = time.time()

    def subscribe_to_topic(self, topic_name):

        if topic_name in self.custom_subscriptions:

            self.get_logger().warn(f"Already subscribed to topic: {topic_name}")
            return

        def callback(msg, topic=topic_name):
            self.messages[topic] = msg

        subscription = self.create_subscription(Joy, topic_name, callback, 10)
        self.custom_subscriptions[topic_name] = subscription
        self.messages[topic_name] = "No data received yet"
        self.get_logger().info(f"Subscribed to topic: {topic_name}")

    def subscribe_to_camera(self, camera_topic):
        if camera_topic in self.custom_subscriptions:
            self.get_logger().warn(f"Already subscribed to camera: {camera_topic}")
            return

        def callback(msg, topic=camera_topic):
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Time -> Ros Time
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().to_msg() #* 1e-9
            now_time = now.sec + now.nanosec #* 1e-9
            latency_ms = (now_time - msg_time) * 1e-9
                                   
            self.camera_frames[topic] = (cv_image, latency_ms)

        subscription = self.create_subscription(RosImage, camera_topic, callback, 10)
        self.custom_subscriptions[camera_topic] = subscription
        self.camera_frames[camera_topic] = None
        self.get_logger().info(f"Subscribed to camera topic: {camera_topic}")

    def joystick_topic_exists(self, topic_name):
        last_seen = self.joystick_last_seen.get(topic_name, 0)
        return (time.time() - last_seen < 1)

    def unsubscribe_from_topic(self, topic_name):
        if topic_name in self.custom_subscriptions:
            self.destroy_subscription(self.custom_subscriptions[topic_name])
            del self.custom_subscriptions[topic_name]

        if topic_name in self.messages:
            del self.messages[topic_name]

        if topic_name in self.camera_frames:
            del self.camera_frames[topic_name]

        self.get_logger().info(f"Unsubscribed from topic: {topic_name}")
 
class TkMultiTopicApp:
    
    def __init__(self, root, ros_node):

        # Initial root and node
        self.root = root
        self.ros_node = ros_node

        # Initialize camera_labels dictionary
        self.camera_labels = {}

        # Set colors
        self.root.configure(bg="white")

        # Joystick check
        self.joystick_status = tk.Label(root, text="Checking joystick topics...", bg="white", fg="black", font=("Arial", 10, "bold"))
        self.joystick_status.place(x=1000, y=10)
        self.root.after(1000, self.check_joystick_topics)

        # Frame 
        self.input_frame = tk.Frame(root, bg="white")
        self.input_frame.grid(row=0, column=0, padx=10, pady=10, sticky="w")

        # Normal topic labels, grid, and buttons
        self.topic_label = tk.Label(self.input_frame, text="Enter Topic Name:", bg="white", fg="red")
        self.topic_label.grid(row=0, column=0, padx=5, pady=5)

        self.topic_entry = tk.Entry(self.input_frame, width=30)
        self.topic_entry.grid(row=0, column=1, padx=5, pady=5)

        self.subscribe_button = tk.Button(self.input_frame, text="Subscribe", command=self.subscribe_to_topic, bg="red", fg="white")
        self.subscribe_button.grid(row=0, column=2, padx=5, pady=5)

        # Create timer
        self.remaining_time = 0
        self.selected_duration = 0
        self.timer_running = False
        self.timer_frame = tk.Frame(root, bg="white")
        self.timer_frame.place(x=650, y=0)
        self.timer_label = tk.Label(self.timer_frame, text=self.format_time(self.remaining_time), font=("Arial", 16, "bold"), fg="red", bg="white")
        self.timer_label.grid(row=0, column=0, columnspan=2, pady=5)
        self.start_timer_button = tk.Button(self.timer_frame, text="Start Match", command=self.start_timer)
        self.start_timer_button.grid(row=1, column=0)
        self.reset_timer_button = tk.Button(self.timer_frame, text="Reset Match", command=self.reset_timer)
        self.reset_timer_button.grid(row=1, column=1)
        self.radio_var = tk.IntVar(value=self.selected_duration)
        tk.Radiobutton(self.timer_frame, text="UCF", variable=self.radio_var, value=900, bg="white", command=self.update_timer_duration).grid(row=2, column=0)
        tk.Radiobutton(self.timer_frame, text="KSC", variable=self.radio_var, value=1800, bg="white", command=self.update_timer_duration).grid(row=2, column=1)

        # Camera topic labels, grid, and buttons
        self.camera_label = tk.Label(self.input_frame, text="Enter Camera Name:", bg="white", fg="red")
        self.camera_label.grid(row=1, column=0, padx=5, pady=5)

        self.camera_entry = tk.Entry(self.input_frame, width=30)
        self.camera_entry.grid(row=1, column=1, padx=5, pady=5)

        self.camera_subscribe_button = tk.Button(self.input_frame, text="Subscribe", command=self.subscribe_to_camera_topic, bg="red", fg="white")
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
        self.root.after(100, self.update_camera_frames)

    # Subscription logic 
    def subscribe_to_topic(self):

        topic_name = self.topic_entry.get().strip()

        if topic_name:

            if topic_name in self.messages_widgets:

                messagebox.showinfo("Already Subscribed", f"Already subscribed to {topic_name}.")
                return

            try:

                self.ros_node.subscribe_to_topic(topic_name)
                self.add_topic_label(topic_name, 100, 100)

            except Exception as e:

                messagebox.showerror("Error", f"Failed to subscribe: {e}")
        else:

            messagebox.showerror("Invalid Input", "Please enter a valid topic name.")

    # Subscription logic 
    def subscribe_to_topic_init(self, topic_name2, x, y, display=True):

        topic_name = topic_name2

        if topic_name:

            if topic_name in self.messages_widgets:

                messagebox.showinfo("Already Subscribed", f"Already subscribed to {topic_name}.")
                return

            try:

                self.ros_node.subscribe_to_topic(topic_name)
                if (display): self.add_topic_label(topic_name, x, y)

            except Exception as e:

                messagebox.showerror("Error", f"Failed to subscribe: {e}")
        else:

            messagebox.showerror("Invalid Input", "Please enter a valid topic name.")

    def subscribe_to_camera_topic(self):

        camera_topic_name = self.camera_entry.get().strip()

        if camera_topic_name:

            if camera_topic_name in self.camera_labels:

                messagebox.showinfo("Already Subscribed", f"Already subscribed to {camera_topic_name}.")
                return
            
            try:
                self.ros_node.subscribe_to_camera(camera_topic_name)
                self.add_camera_label(camera_topic_name, 100, 100)

            except Exception as e:

                messagebox.showerror("Error", f"Failed to subscribe: {e}")

        else:

            messagebox.showerror("Invalid Input", "Please enter a valid camera topic name.")

    def subscribe_to_camera_topic_init(self, camera_topic, x, y):

        camera_topic_name = camera_topic

        if camera_topic_name:

            if camera_topic_name in self.camera_labels:

                messagebox.showinfo("Already Subscribed", f"Already subscribed to {camera_topic_name}.")
                return
            
            try:
                self.ros_node.subscribe_to_camera(camera_topic_name)
                self.add_camera_label(camera_topic_name, x, y)

            except Exception as e:

                messagebox.showerror("Error", f"Failed to subscribe: {e}")

        else:

            messagebox.showerror("Invalid Input", "Please enter a valid camera topic name.")

    def check_joystick_topics(self):

        joy1_detected = self.ros_node.joystick_topic_exists(joystick_topic_1)
        joy2_detected = self.ros_node.joystick_topic_exists(joystick_topic_2)

        status_text = f"Joystick 1: {'✔' if joy1_detected else '✖'} | Joystick 2: {'✔' if joy2_detected else '✖'}"
        if (joy1_detected and joy2_detected):
            status_color = "green"
        elif (joy1_detected or joy2_detected):
            status_color = "black"
        else:
            status_color = "red"

        self.joystick_status.config(text=status_text, fg=status_color)

        # Re-check 
        self.root.after(3000, self.check_joystick_topics)


    # Labeling logic
    def add_topic_label(self, topic_name, x, y):
        
        label_frame = tk.Frame(self.messages_frame, bg="white", bd=1, relief="solid")
        label_frame.place(x=x, y=y) 
        label_topic = tk.Label(label_frame, text=f"Topic: {topic_name}", font=("Arial", 10, "bold"))
        label_topic.pack(side="top", padx=5, pady=2)
        label_message = tk.Label(label_frame, text="No data received yet", font=("Arial", 12), compound="bottom")
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

    def add_camera_label(self, camera_topic_name, x, y):
 
        label_frame = tk.Frame(self.messages_frame, bg="lightblue", bd=1, relief="solid")
        label_frame.place(x=x, y=y)
        label_camera = tk.Label(label_frame, text=f"Camera: {camera_topic_name}", font=("Arial", 10, "bold"))
        label_camera.pack(side="top", padx=5, pady=2)
        label_message = tk.Label(label_frame, text="No data received yet", font=("Arial", 12))
        label_message.pack(side="top", padx=5, pady=2)

        # Store message
        self.camera_labels[camera_topic_name] = label_message
        
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
 
            if self.messages_widgets[topic_name]["timeout_timer"]:
                self.messages_widgets[topic_name]["timeout_timer"].cancel()
      
            self.messages_widgets[topic_name]["frame"].destroy()
            del self.messages_widgets[topic_name]
         
        if topic_name in self.camera_labels:
            del self.camera_labels[topic_name]
    
        # Unsubscribe
        self.ros_node.unsubscribe_from_topic(topic_name)

 
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
                    
    def update_camera_frames(self):
     
        for topic, data in list(self.ros_node.camera_frames.items()):
         
            if data is not None and topic in self.camera_labels:
             
                frame, latency_ms = data
                cv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(cv_image)
                img_tk = ImageTk.PhotoImage(image=img)

                self.camera_labels[topic].config(
                    image=img_tk,
                    compound="bottom",
                    text=f"Latency: {latency_ms:.1f} ms"
                )
                self.camera_labels[topic].image = img_tk

        # Schedule the next update on the main thread
        self.root.after(100, self.update_camera_frames)
 
    def check_data_timeout(self, topic_name, label_message):
 
        def timeout_action():
            if label_message.cget("text") != "No data received yet":
                label_message.config(text="Data timed out")

        timer = threading.Timer(3, timeout_action)
        self.messages_widgets[topic_name]["timeout_timer"] = timer
        timer.start()

    def format_time(self, seconds):
        return f"{seconds // 60}:{seconds % 60:02}"

    def update_timer_duration(self):
        self.selected_duration = self.radio_var.get()
        self.remaining_time = self.selected_duration
        self.timer_label.config(text=self.format_time(self.remaining_time))

    def start_timer(self):
        if not self.timer_running:
            self.timer_running = True
            self.countdown()

    def reset_timer(self):
        self.timer_running = False
        self.remaining_time = self.selected_duration
        self.timer_label.config(text=self.format_time(self.remaining_time))

    def countdown(self):
        if self.timer_running and self.remaining_time > 0:
            self.remaining_time -= 1
            self.timer_label.config(text=self.format_time(self.remaining_time))
            self.root.after(1000, self.countdown)
        elif self.remaining_time == 0:
            self.timer_running = False
            self.timer_label.config(text="Time's up!")

def main():

    # Main loop
    rclpy.init()
    ros_node = MultiTopicSubscriber()
    root = tk.Tk()
    root.title("Astrobotics GUI")
    root.geometry("800x600")  
    app = TkMultiTopicApp(root, ros_node)

    # Declare initial topics and subscribe
    initial_topics = [(joystick_topic_1, 100, 100), (joystick_topic_2, 300, 100)]
    initial_cameras = [("/driver/selected_image", 100, 300)]
    #initial_cameras = [("usbcam_image_2", 600, 600)]
    
    for topic_name, x, y in initial_topics:
        threading.Thread(target=app.subscribe_to_topic_init, args=(topic_name, x, y, False), daemon=True).start()
    
    for camera_name, x, y in initial_cameras:
        threading.Thread(target=app.subscribe_to_camera_topic_init, args=(camera_name, x, y), daemon=True).start()
     
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
