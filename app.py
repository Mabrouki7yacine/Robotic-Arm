import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import time
import math
import socket
import struct
import threading
import queue

class TCPSender:
    def __init__(self, port=3000):
        """
        Initialize the TCP sender.
        
        Args:
            port (int): Port number to use for TCP communication
        """
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.connected = False
        self.send_queue = queue.Queue()
        
    def start_server(self):
        """Start the TCP server in a separate thread."""
        self.running = True
        self.server_thread = threading.Thread(target=self._server_thread)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # Start the sender thread
        self.sender_thread = threading.Thread(target=self._sender_thread)
        self.sender_thread.daemon = True
        self.sender_thread.start()
        
    def _server_thread(self):
        """Server thread function to handle connections."""
        try:
            print("Starting TCP server, waiting for connection...")
            server_address = ("", self.port)
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(server_address)
            self.server_socket.listen(4)
            
            while self.running:
                try:
                    self.client_socket, client_address = self.server_socket.accept()
                    print(f"Connection established from: {client_address}")
                    self.connected = True
                    
                    # Wait for the client to disconnect
                    while self.connected and self.running:
                        time.sleep(0.1)
                        
                except socket.timeout:
                    continue
                except Exception as e:
                    print(f"Connection error: {e}")
                    self.connected = False
                    
                # Reset for next connection
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
                    
        except Exception as e:
            print(f"Server thread error: {e}")
        finally:
            self.cleanup()
            
    def _sender_thread(self):
        """Thread to send angle data from the queue."""
        while self.running:
            try:
                if not self.connected or self.client_socket is None:
                    time.sleep(0.1)
                    continue
                    
                try:
                    # Get data with timeout to allow checking running status
                    angles = self.send_queue.get(timeout=0.1)
                    
                    # Send the final target angles
                    if self.connected and self.client_socket:
                        theta1_deg = int(np.degrees(angles[0]))
                        # Send the absolute value of theta2 degrees (e.g., 115.78°)
                        theta2_deg = int(abs(np.degrees(angles[1])))
                        # Pack as two integers
                        packed_data = struct.pack('ii', theta1_deg, theta2_deg)
                        self.client_socket.sendall(packed_data)
                        print(f"[Sent] FINAL TARGET ANGLES: theta1={theta1_deg}°, theta2={theta2_deg}°")
                        
                except queue.Empty:
                    # Queue empty, continue loop
                    continue
                    
            except Exception as e:
                print(f"Sender error: {e}")
                self.connected = False
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except:
                        pass
                    self.client_socket = None
                time.sleep(0.1)
                
    def send_angles(self, theta1, theta2):
        """
        Add angles to the send queue.
        
        Args:
            theta1 (float): First joint angle in radians
            theta2 (float): Second joint angle in radians
        """
        # Clear the queue first to make sure we only have the most recent angles
        while not self.send_queue.empty():
            try:
                self.send_queue.get_nowait()
            except queue.Empty:
                break
                
        # Add the new target angles to the queue
        self.send_queue.put((theta1, theta2))
                
    def cleanup(self):
        """Clean up sockets and threads."""
        self.running = False
        self.connected = False
        
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
                
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

class RoboticArm:
    def __init__(self, l1, l2):
        """
        Initialize the robotic arm with two segments.
        
        Args:
            l1 (float): Length of the first segment in cm
            l2 (float): Length of the second segment in cm
        """
        self.l1 = l1
        self.l2 = l2
        
        # Starting position (angles in radians)
        self.theta1 = np.pi/2  # 90 degrees
        self.theta2 = 0.0      # 0 degrees
        
        # Keep track of target and whether it's reachable
        self.target = None
        self.reachable = True
        
        # Flag to indicate if we are animating
        self.is_animating = False
        
        # Final target angles
        self.target_theta1 = None
        self.target_theta2 = None
        
        # Create TCP sender
        self.tcp_sender = TCPSender()
        self.tcp_sender.start_server()
        
    def forward_kinematics(self, theta1, theta2):
        """
        Calculate end position given joint angles.
        
        Args:
            theta1 (float): First joint angle in radians
            theta2 (float): Second joint angle in radians
            
        Returns:
            tuple: (y, z) coordinates of the end effector
        """
        # Position of first joint (always at origin)
        j1_pos = (0, 0)
        
        # Position of second joint
        j2_y = self.l1 * np.cos(theta1)
        j2_z = self.l1 * np.sin(theta1)
        j2_pos = (j2_y, j2_z)
        
        # Position of end effector
        ee_y = j2_y + self.l2 * np.cos(theta1 + theta2)
        ee_z = j2_z + self.l2 * np.sin(theta1 + theta2)
        ee_pos = (ee_y, ee_z)
        
        return j1_pos, j2_pos, ee_pos
    
    def inverse_kinematics(self, y, z):
        """
        Calculate joint angles given target end effector position.
        Prefers the elbow-up configuration.
        
        Args:
            y (float): Target y coordinate in cm
            z (float): Target z coordinate in cm
            
        Returns:
            tuple: (theta1, theta2) joint angles in radians
            bool: Whether the target is reachable
        """
        # Ensure positive coordinates
        y = abs(y)
        z = abs(z)
        
        # Save target
        self.target = (y, z)
        
        # Calculate distance from base to target
        distance = np.sqrt(y**2 + z**2)
        
        # Check if the target is reachable
        max_reach = self.l1 + self.l2
        min_reach = abs(self.l1 - self.l2)
        
        if distance > max_reach or distance < min_reach:
            self.reachable = False
            # If unreachable, aim in the direction of the target but at max reach
            if distance > max_reach:
                angle = np.arctan2(z, y)
                y = max_reach * np.cos(angle) * 0.99  # Slightly under max reach
                z = max_reach * np.sin(angle) * 0.99
            # If too close, extend to min reach
            else:
                angle = np.arctan2(z, y)
                y = min_reach * np.cos(angle) * 1.01  # Slightly over min reach
                z = min_reach * np.sin(angle) * 1.01
        else:
            self.reachable = True
        
        # Law of cosines to get theta2
        cos_theta2 = (y**2 + z**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # Clamp to avoid numerical errors
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        
        # Use negative theta2 for elbow-up configuration
        theta2 = -np.arccos(cos_theta2)
        
        # Get theta1 using atan2
        k1 = self.l1 + self.l2 * np.cos(theta2)
        k2 = self.l2 * np.sin(theta2)
        theta1 = np.arctan2(z, y) - np.arctan2(k2, k1)
        
        # Normalize angles
        theta1 = (theta1 + 2*np.pi) % (2*np.pi)
        #theta2 = (theta2 + 2*np.pi) % (2*np.pi)
        theta2 = (theta2 + 2*np.pi)
        
        # Store target angles for TCP sending
        self.target_theta1 = theta1
        self.target_theta2 = theta2
        
        return theta1, theta2, self.reachable
    
    def get_arm_positions(self):
        """
        Get current positions of the joints and end effector.
        
        Returns:
            tuple: ((j1_y, j1_z), (j2_y, j2_z), (ee_y, ee_z))
        """
        return self.forward_kinematics(self.theta1, self.theta2)
    
    def set_angles(self, theta1, theta2):
        """
        Set the joint angles without sending via TCP.
        Only updates the internal state for animation.
        
        Args:
            theta1 (float): First joint angle in radians
            theta2 (float): Second joint angle in radians
        """
        self.theta1 = theta1
        self.theta2 = theta2
        
    def send_target_angles(self):
        """
        Send the final target angles via TCP.
        Only called once animation is complete.
        """
        if self.target_theta1 is not None and self.target_theta2 is not None:
            self.tcp_sender.send_angles(self.target_theta1, self.target_theta2)
            # Use absolute value for theta2 in the log message to match what's shown in GUI
            print(f"Sending FINAL target angles: theta1={np.degrees(self.target_theta1):.1f}°, theta2={abs(np.degrees(self.target_theta2)):.1f}°")

class ArmVisualizer:
    def __init__(self, arm, fig_size=(8, 8)):
        """
        Initialize the arm visualizer.
        
        Args:
            arm (RoboticArm): The robotic arm to visualize
            fig_size (tuple): Figure size (width, height)
        """
        self.arm = arm
        self.fig, self.ax = plt.subplots(figsize=fig_size)
        self.setup_plot()
        
        # For animation
        self.animation_frames = 60
        self.theta1_start = arm.theta1
        self.theta2_start = arm.theta2
        self.theta1_target = arm.theta1
        self.theta2_target = arm.theta2
        
    def setup_plot(self):
        """Set up the plot for visualization."""
        # Set limits based on arm length - only positive quadrant
        max_length = self.arm.l1 + self.arm.l2
        self.ax.set_xlim(-1, max_length * 1.2)
        self.ax.set_ylim(-1, max_length * 1.2)
        
        # Labels and title
        self.ax.set_xlabel('Y (cm)')
        self.ax.set_ylabel('Z (cm)')
        self.ax.set_title('2-Joint Robotic Arm Simulation (Positive Y-Z Only)')
        
        # Grid and aspect ratio
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        
        # Create arm segments (lines will be updated later)
        self.segment1, = self.ax.plot([], [], 'o-', lw=4, markersize=10, color='blue')
        self.segment2, = self.ax.plot([], [], 'o-', lw=4, markersize=10, color='green')
        
        # Create target point
        self.target_point, = self.ax.plot([], [], 'ro', markersize=10)
        
        # Create reach circle (partial circle only in positive quadrant)
        max_reach = self.arm.l1 + self.arm.l2
        min_reach = abs(self.arm.l1 - self.arm.l2)
        
        # Create partial circles for the positive quadrant
        theta = np.linspace(0, np.pi/2, 100)
        outer_x = max_reach * np.cos(theta)
        outer_y = max_reach * np.sin(theta)
        inner_x = min_reach * np.cos(theta)
        inner_y = min_reach * np.sin(theta)
        
        # Outer reach arc (dashed)
        self.ax.plot(outer_x, outer_y, linestyle='--', color='gray', alpha=0.5)
        
        # Inner reach arc (dashed)
        self.ax.plot(inner_x, inner_y, linestyle='--', color='gray', alpha=0.5)
        
        # Status text
        self.status_text = self.ax.text(0.02, 0.02, '', transform=self.ax.transAxes, 
                                       fontsize=10, color='red')
        
        # Angles text
        self.angles_text = self.ax.text(0.02, 0.96, '', transform=self.ax.transAxes, 
                                       fontsize=10, verticalalignment='top')
        
        # Connection status
        self.connection_text = self.ax.text(0.98, 0.02, 'TCP: Waiting', transform=self.ax.transAxes,
                                          fontsize=10, horizontalalignment='right', color='blue')
        
    def update_plot(self):
        """Update the plot with current arm position."""
        j1_pos, j2_pos, ee_pos = self.arm.get_arm_positions()
        
        # Update segment positions
        self.segment1.set_data([j1_pos[0], j2_pos[0]], [j1_pos[1], j2_pos[1]])
        self.segment2.set_data([j2_pos[0], ee_pos[0]], [j2_pos[1], ee_pos[1]])
        
        # Update target if available
        if self.arm.target:
            self.target_point.set_data([self.arm.target[0]], [self.arm.target[1]])
            
            # Update status text
            if not self.arm.reachable:
                self.status_text.set_text('Target out of reach!')
                self.status_text.set_color('red')
            else:
                self.status_text.set_text('Target reachable')
                self.status_text.set_color('green')
        
        # Update angles text - Show absolute value for theta2
        theta1_deg = np.degrees(self.arm.theta1)
        theta2_deg = abs(np.degrees(self.arm.theta2))  # Absolute value
        self.angles_text.set_text(f'Joint 1: {theta1_deg:.1f}°\nJoint 2: {theta2_deg:.1f}°')
        
        # Update connection status
        if self.arm.tcp_sender.connected:
            self.connection_text.set_text('TCP: Connected')
            self.connection_text.set_color('green')
        else:
            self.connection_text.set_text('TCP: Waiting')
            self.connection_text.set_color('blue')
        
        self.fig.canvas.draw_idle()
    
    def animate_to_target(self, y, z):
        """
        Use manual animation with frames.
        
        Args:
            y (float): Target y coordinate in cm
            z (float): Target z coordinate in cm
        """
        # Ensure positive coordinates
        y = abs(y)
        z = abs(z)
        
        # Calculate target angles
        self.theta1_start = self.arm.theta1
        self.theta2_start = self.arm.theta2
        self.theta1_target, self.theta2_target, _ = self.arm.inverse_kinematics(y, z)
        
        # Normalize angle changes to take the shortest path
        dtheta1 = (self.theta1_target - self.theta1_start + np.pi) % (2*np.pi) - np.pi
        dtheta2 = (self.theta2_target - self.theta2_start + np.pi) % (2*np.pi) - np.pi
        
        self.theta1_target = self.theta1_start + dtheta1
        self.theta2_target = self.theta2_start + dtheta2
        
        # Flag that animation has started
        self.arm.is_animating = True

        # Manual animation
        frames = 30
        for frame in range(frames + 1):
            t = frame / frames  # Normalized time [0, 1]
            
            # Use easing function for smoother motion
            t = self.ease_in_out(t)
            
            theta1 = self.theta1_start + (self.theta1_target - self.theta1_start) * t
            theta2 = self.theta2_start + (self.theta2_target - self.theta2_start) * t
            
            # Update arm angles (without sending TCP)
            self.arm.set_angles(theta1, theta2)
            
            # Update plot
            self.update_plot()
            
            # Process GUI events
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            time.sleep(0.02)  # Short delay
        
        # Animation complete, now send final target angles via TCP
        self.arm.is_animating = False
        self.arm.send_target_angles()
    
    @staticmethod
    def ease_in_out(t):
        """Simple easing function for smoother animation."""
        return t * t * (3 - 2 * t)

class ArmGUI:
    def __init__(self, root, arm):
        """
        Initialize the GUI for the robotic arm.
        
        Args:
            root (tk.Tk): The root Tkinter window
            arm (RoboticArm): The robotic arm to control
        """
        self.root = root
        self.arm = arm
        self.visualizer = ArmVisualizer(arm)
        
        # Set up the GUI
        self.root.title("Robotic Arm Controller with TCP")
        
        # Create a frame for input controls
        input_frame = tk.Frame(root)
        input_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=10)
        
        # Y coordinate input
        tk.Label(input_frame, text="Y coordinate (cm):").grid(row=0, column=0, padx=5, pady=5)
        self.y_var = tk.DoubleVar(value=10.0)
        self.y_entry = tk.Entry(input_frame, textvariable=self.y_var, width=8)
        self.y_entry.grid(row=0, column=1, padx=5, pady=5)
        
        # Z coordinate input
        tk.Label(input_frame, text="Z coordinate (cm):").grid(row=0, column=2, padx=5, pady=5)
        self.z_var = tk.DoubleVar(value=10.0)
        self.z_entry = tk.Entry(input_frame, textvariable=self.z_var, width=8)
        self.z_entry.grid(row=0, column=3, padx=5, pady=5)
        
        # Move button
        self.move_button = tk.Button(input_frame, text="Move Arm", command=self.move_arm)
        self.move_button.grid(row=0, column=4, padx=10, pady=5)
        
        # Reset button
        self.reset_button = tk.Button(input_frame, text="Reset", command=self.reset_arm)
        self.reset_button.grid(row=0, column=5, padx=10, pady=5)
        
        # Add port information
        port_text = f"TCP server running on port {self.arm.tcp_sender.port}"
        port_label = tk.Label(input_frame, text=port_text, font=("Arial", 8), fg="blue")
        port_label.grid(row=1, column=0, columnspan=3, padx=5, pady=0, sticky="w")
        
        # Add note about positive values
        note_text = "Note: Negative values will be converted to positive"
        note_label = tk.Label(input_frame, text=note_text, font=("Arial", 8), fg="gray")
        note_label.grid(row=1, column=3, columnspan=3, padx=5, pady=0, sticky="w")
        
        # Embed matplotlib figure
        self.canvas = FigureCanvasTkAgg(self.visualizer.fig, master=root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Configure matplotlib for interactive use
        plt.ion()
        
        # Initial plot update
        self.visualizer.update_plot()
        
        # Start a periodic update to check TCP connection status
        self.update_connection_status()
    
    def update_connection_status(self):
        """Periodically update the connection status in the plot."""
        self.visualizer.update_plot()
        # Schedule next update
        self.root.after(1000, self.update_connection_status)
    
    def move_arm(self):
        """Move the arm to the target position."""
        try:
            # Disable buttons during animation
            self.move_button.config(state=tk.DISABLED)
            self.reset_button.config(state=tk.DISABLED)
            
            # Get target coordinates (convert to positive)
            y = abs(self.y_var.get())
            z = abs(self.z_var.get())
            
            # Update the entry fields with positive values
            self.y_var.set(y)
            self.z_var.set(z)
            
            # Use the manual animation method
            self.visualizer.animate_to_target(y, z)
            
            # Re-enable buttons
            self.move_button.config(state=tk.NORMAL)
            self.reset_button.config(state=tk.NORMAL)
            
        except Exception as e:
            print(f"Error: {e}")
            # Make sure buttons are re-enabled
            self.move_button.config(state=tk.NORMAL)
            self.reset_button.config(state=tk.NORMAL)
    
    def reset_arm(self):
        """Reset the arm to the default position."""
        try:
            # Disable buttons during animation
            self.move_button.config(state=tk.DISABLED)
            self.reset_button.config(state=tk.DISABLED)
            
            # Reset to default position
            self.visualizer.animate_to_target(10.0, 10.0)
            
            # Re-enable buttons
            self.move_button.config(state=tk.NORMAL)
            self.reset_button.config(state=tk.NORMAL)
            
        except Exception as e:
            print(f"Error: {e}")
            # Make sure buttons are re-enabled
            self.move_button.config(state=tk.NORMAL)
            self.reset_button.config(state=tk.NORMAL)

def main():
    """Main function to run the application."""
    # Create robotic arm with segment lengths in cm
    arm_length1 = 12.5  # First segment length (cm)
    arm_length2 = 14  # Second segment length (cm)
    arm = RoboticArm(arm_length1, arm_length2)
    
    # Create GUI
    root = tk.Tk()
    app = ArmGUI(root, arm)
    
    # Start the main loop
    try:
        root.mainloop()
    finally:
        # Make sure to clean up the TCP sender
        if arm.tcp_sender:
            arm.tcp_sender.cleanup()

if __name__ == "__main__":
    main()