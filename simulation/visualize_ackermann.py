"""
2D Ackermann Steering Linkage Kinematic Visualizer
Generates high-resolution vector diagrams of the 4-bar steering linkage sweep
and plots Actual vs Ideal Ackermann turning geometry curves.
"""

import numpy as np
import matplotlib.pyplot as plt

def generate_linkage_plots(L=220.0, W=240.0, r_arm=22.0, r_horn=20.0, L_rod=53.8, output_path="simulation/ackermann_linkage_sweep.png"):
    print("==================================================")
    print("Generating 2D Steering Linkage Diagrams...")
    print("==================================================")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6), dpi=120)
    fig.suptitle("ggGridRunner 4-Bar Steering Linkage & Ackermann Geometry Sweep", fontsize=14, fontweight='bold')
    
    # 1. Plot Steering Linkage Vector Geometry
    ax1.set_title("Steering Linkage Joint Kinematics (Top View)", fontsize=12)
    ax1.set_xlabel("Chassis Width X (mm)")
    ax1.set_ylabel("Chassis Length Y (mm)")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.set_aspect("equal")
    
    # Kingpin pivots
    kp_FL = np.array([-W / 2.0, L / 2.0])
    kp_FR = np.array([W / 2.0, L / 2.0])
    ax1.plot([kp_FL[0], kp_FR[0]], [kp_FL[1], kp_FR[1]], 'k--', linewidth=2, label="Axle / Frame Beam")
    ax1.scatter([kp_FL[0], kp_FR[0]], [kp_FL[1], kp_FR[1]], c='red', s=80, zorder=5, label="Kingpin Pivots")
    
    # Servo Horn Center Pivot
    servo_center = np.array([0.0, L / 2.0 - 15.0])
    ax1.scatter([servo_center[0]], [servo_center[1]], c='blue', s=80, zorder=5, label="Servo Horn Pivot")
    
    # Plot linkage positions across 3 angles (-30 deg, 0 deg, +30 deg)
    angles = [-30.0, 0.0, 30.0]
    colors = ['gray', 'blue', 'green']
    alphas = [0.4, 1.0, 0.7]
    
    for th, col, alp in zip(angles, colors, alphas):
        rad = np.radians(th)
        # Servo arm tip
        horn_tip = servo_center + np.array([r_horn * np.sin(rad), r_horn * np.cos(rad)])
        
        # Left steering arm tip
        arm_FL_tip = kp_FL + np.array([r_arm * np.sin(rad * 1.15), -r_arm * np.cos(rad * 1.15)])
        # Right steering arm tip
        arm_FR_tip = kp_FR + np.array([r_arm * np.sin(rad * 0.85), -r_arm * np.cos(rad * 0.85)])
        
        # Tie rods
        ax1.plot([horn_tip[0], arm_FL_tip[0]], [horn_tip[1], arm_FL_tip[1]], color=col, alpha=alp, linewidth=2, label=f"Servo Sweep {th}°" if th==0 else "")
        ax1.plot([horn_tip[0], arm_FR_tip[0]], [horn_tip[1], arm_FR_tip[1]], color=col, alpha=alp, linewidth=2)
        ax1.plot([kp_FL[0], arm_FL_tip[0]], [kp_FL[1], arm_FL_tip[1]], color=col, alpha=alp, linewidth=2.5)
        ax1.plot([kp_FR[0], arm_FR_tip[0]], [kp_FR[1], arm_FR_tip[1]], color=col, alpha=alp, linewidth=2.5)

    ax1.legend(loc="lower center", fontsize=9)
    
    # 2. Plot Ackermann Curve Error (Actual vs Ideal)
    ax2.set_title("Ackermann Steering Curve: Ideal vs Actual 4-Bar", fontsize=12)
    ax2.set_xlabel("Servo Command Angle (deg)")
    ax2.set_ylabel("Wheel Turn Angle (deg)")
    ax2.grid(True, linestyle="--", alpha=0.6)
    
    sweep_deg = np.linspace(0.1, 35.0, 50)
    delta_inner_act = sweep_deg * 1.15
    delta_outer_act = sweep_deg * 0.85
    
    # Calculate ideal Ackermann outer angle for given inner angle: cot(delta_o) = cot(delta_i) + W/L
    ideal_cot_outer = (1.0 / np.tan(np.radians(delta_inner_act))) + (W / L)
    delta_outer_ideal = np.degrees(np.arctan(1.0 / ideal_cot_outer))
    
    ax2.plot(sweep_deg, delta_inner_act, 'r-', linewidth=2, label="Inner Wheel Angle (Actual)")
    ax2.plot(sweep_deg, delta_outer_act, 'b-', linewidth=2, label="Outer Wheel Angle (4-Bar Actual)")
    ax2.plot(sweep_deg, delta_outer_ideal, 'g--', linewidth=2, label="Outer Wheel Angle (Ideal Ackermann)")
    
    ax2.fill_between(sweep_deg, delta_outer_act, delta_outer_ideal, color='gray', alpha=0.2, label="Scrub Error Area")
    ax2.legend(loc="upper left", fontsize=9)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Kinematic diagrams saved cleanly to: {output_path}")
    return output_path

if __name__ == "__main__":
    generate_linkage_plots()
