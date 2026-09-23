"""
2D Mechanical Steering Linkage Animation Generator
Renders a clean top-view mechanical drawing animation of the chassis, kingpins,
servo horn, tie-rods, and front wheels sweeping through Ackermann turning angles.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def generate_linkage_animation(L=220.0, W=240.0, r_arm=22.0, r_horn=20.0, L_rod=53.8, output_path="simulation/steering_linkage_animation.gif"):
    print("==================================================")
    print("Generating 2D Mechanical Linkage Animation...")
    print("==================================================")
    
    fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
    ax.set_title("ggGridRunner 4-Bar Steering Linkage Mechanical Motion", fontsize=12, fontweight='bold')
    ax.set_xlabel("Chassis Width X (mm)")
    ax.set_ylabel("Chassis Length Y (mm)")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.set_aspect("equal")
    ax.set_xlim(-160, 160)
    ax.set_ylim(40, 200)
    
    # Static Chassis Geometry
    kp_FL = np.array([-W / 2.0, L / 2.0])
    kp_FR = np.array([W / 2.0, L / 2.0])
    servo_center = np.array([0.0, L / 2.0 - 25.0])
    
    # Plot frame beam & pivots
    ax.plot([kp_FL[0], kp_FR[0]], [kp_FL[1], kp_FR[1]], 'k-', linewidth=4, label="Front Axle Beam")
    ax.plot([servo_center[0], 0], [servo_center[1], L / 2.0], 'k:', linewidth=2, label="Chassis Centerline")
    ax.scatter([kp_FL[0], kp_FR[0]], [kp_FL[1], kp_FR[1]], c='red', s=100, zorder=5, label="Kingpin Pivots")
    ax.scatter([servo_center[0]], [servo_center[1]], c='blue', s=100, zorder=5, label="Servo Pivot")
    
    # Dynamic Elements
    line_horn, = ax.plot([], [], 'r-', linewidth=4, label="Servo Horn")
    line_tie_FL, = ax.plot([], [], 'y-', linewidth=3, label="FL Tie-Rod")
    line_tie_FR, = ax.plot([], [], 'y-', linewidth=3, label="FR Tie-Rod")
    line_arm_FL, = ax.plot([], [], 'g-', linewidth=4, label="FL Steering Arm")
    line_arm_FR, = ax.plot([], [], 'g-', linewidth=4, label="FR Steering Arm")
    
    # Wheels
    poly_wheel_FL, = ax.plot([], [], 'k-', linewidth=6, label="FL Wheel")
    poly_wheel_FR, = ax.plot([], [], 'k-', linewidth=6, label="FR Wheel")
    
    ax.legend(loc="lower right", fontsize=8)
    
    frames = 60
    angles = np.sin(np.linspace(0, 2 * np.pi, frames)) * 30.0
    
    def animate(i):
        th = angles[i]
        rad = np.radians(th)
        
        # Servo horn tip position
        horn_tip = servo_center + np.array([r_horn * np.sin(rad), r_horn * np.cos(rad)])
        line_horn.set_data([servo_center[0], horn_tip[0]], [servo_center[1], horn_tip[1]])
        
        # Wheel angles
        delta_i = rad * 1.15
        delta_o = rad * 0.85
        
        arm_FL_tip = kp_FL + np.array([r_arm * np.sin(delta_i), -r_arm * np.cos(delta_i)])
        arm_FR_tip = kp_FR + np.array([r_arm * np.sin(delta_o), -r_arm * np.cos(delta_o)])
        
        line_arm_FL.set_data([kp_FL[0], arm_FL_tip[0]], [kp_FL[1], arm_FL_tip[1]])
        line_arm_FR.set_data([kp_FR[0], arm_FR_tip[0]], [kp_FR[1], arm_FR_tip[1]])
        
        line_tie_FL.set_data([horn_tip[0], arm_FL_tip[0]], [horn_tip[1], arm_FL_tip[1]])
        line_tie_FR.set_data([horn_tip[0], arm_FR_tip[0]], [horn_tip[1], arm_FR_tip[1]])
        
        # Draw wheel orientation bars
        w_len = 35.0
        w_FL_dx = (w_len / 2.0) * np.sin(delta_i)
        w_FL_dy = (w_len / 2.0) * np.cos(delta_i)
        poly_wheel_FL.set_data([kp_FL[0] - w_FL_dx, kp_FL[0] + w_FL_dx], [kp_FL[1] - w_FL_dy, kp_FL[1] + w_FL_dy])
        
        w_FR_dx = (w_len / 2.0) * np.sin(delta_o)
        w_FR_dy = (w_len / 2.0) * np.cos(delta_o)
        poly_wheel_FR.set_data([kp_FR[0] - w_FR_dx, kp_FR[0] + w_FR_dx], [kp_FR[1] - w_FR_dy, kp_FR[1] + w_FR_dy])
        
        return line_horn, line_tie_FL, line_tie_FR, line_arm_FL, line_arm_FR, poly_wheel_FL, poly_wheel_FR
    
    anim = animation.FuncAnimation(fig, animate, frames=frames, interval=50, blit=True)
    anim.save(output_path, writer='pillow', fps=20)
    plt.close()
    print(f"2D Mechanical Linkage Animation saved cleanly to: {output_path}")

if __name__ == "__main__":
    generate_linkage_animation()
