#!/usr/bin/env python3
# tools/simulate_with_diagnostics.py
# Enhanced simulation with timing control and diagnostic plotting
import subprocess
import time
import signal
import sys
import os
import json
from datetime import datetime

def signal_handler(signum, frame):
    print(f"\n[simulate_with_diagnostics] Received signal {signum}, shutting down gracefully...")
    sys.exit(0)

def run_simulation_with_diagnostics(duration_seconds=30):
    """Run simulation for specified duration and generate diagnostic plots"""
    
    # Set up signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print(f"[simulate_with_diagnostics] Starting simulation for {duration_seconds} seconds...")
    
    # Create output directory for logs
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"simulation_logs_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)
    
    # Clean up any existing processes first
    print("[simulate_with_diagnostics] Cleaning up any existing processes...")
    subprocess.run(["pkill", "-f", "simulate_leader.py|host_bridge_udp_mirror.py|virtual_follower.py"], 
                   capture_output=True)
    time.sleep(1)
    
    # Start the simulation processes
    processes = []
    try:
        # Start virtual follower with logging
        print("[simulate_with_diagnostics] Starting virtual follower...")
        vf_cmd = ["python", "tools/virtual_follower.py"]
        vf_process = subprocess.Popen(vf_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(("virtual_follower", vf_process))
        
        time.sleep(1.0)  # Give more time for port binding
        
        # Start host bridge with logging
        print("[simulate_with_diagnostics] Starting host bridge...")
        hb_cmd = ["python", "tools/host_bridge_udp_mirror.py"]
        hb_process = subprocess.Popen(hb_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(("host_bridge", hb_process))
        
        time.sleep(1.0)  # Give more time for port binding
        
        # Start simulated leader with logging
        print("[simulate_with_diagnostics] Starting simulated leader...")
        sl_cmd = ["python", "tools/simulate_leader.py"]
        sl_process = subprocess.Popen(sl_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        processes.append(("simulate_leader", sl_process))
        
        print(f"[simulate_with_diagnostics] All processes started. Running for {duration_seconds} seconds...")
        
        # Run for specified duration
        start_time = time.time()
        while time.time() - start_time < duration_seconds:
            time.sleep(0.1)
            elapsed = time.time() - start_time
            if elapsed % 5 < 0.1:  # Print progress every 5 seconds
                print(f"[simulate_with_diagnostics] Elapsed: {elapsed:.1f}s / {duration_seconds}s")
        
        print(f"[simulate_with_diagnostics] Duration complete. Stopping processes...")
        
    except KeyboardInterrupt:
        print("\n[simulate_with_diagnostics] Interrupted by user")
    finally:
        # Stop all processes
        for name, process in processes:
            print(f"[simulate_with_diagnostics] Stopping {name}...")
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                print(f"[simulate_with_diagnostics] Force killing {name}...")
                process.kill()
        
        # Collect output data
        print(f"[simulate_with_diagnostics] Collecting diagnostic data...")
        collect_diagnostic_data(processes, output_dir)
        
        # Generate plots
        print(f"[simulate_with_diagnostics] Generating diagnostic plots...")
        generate_diagnostic_plots(output_dir)
        
        print(f"[simulate_with_diagnostics] Complete! Check {output_dir}/ for results")

def collect_diagnostic_data(processes, output_dir):
    """Collect and save diagnostic data from process outputs"""
    
    # Save raw outputs
    for name, process in processes:
        stdout, stderr = process.communicate()
        
        with open(f"{output_dir}/{name}_stdout.log", "w") as f:
            f.write(stdout)
        with open(f"{output_dir}/{name}_stderr.log", "w") as f:
            f.write(stderr)
    
    # Parse virtual follower data for analysis
    parse_follower_data(output_dir)

def parse_follower_data(output_dir):
    """Parse virtual follower output to extract timing and position data"""
    import re
    
    follower_log = f"{output_dir}/virtual_follower_stdout.log"
    if not os.path.exists(follower_log):
        return
    
    positions = []
    timestamps = []
    modes = []
    
    with open(follower_log, 'r') as f:
        for line in f:
            # Parse FOLLOW commands: [virtual_follower] FOLLOW q=(0.123, -0.456, 0.789, 0.15) t_us=123456789
            follow_match = re.search(r'FOLLOW q=\(([^)]+)\) t_us=(\d+)', line)
            if follow_match:
                q_str = follow_match.group(1)
                t_us = int(follow_match.group(2))
                
                # Parse joint positions
                q_values = [float(x.strip()) for x in q_str.split(',')]
                positions.append(q_values)
                timestamps.append(t_us)
            
            # Parse MODE commands: [virtual_follower] MODE=FOLLOW
            mode_match = re.search(r'MODE=(\w+)', line)
            if mode_match:
                modes.append({
                    'mode': mode_match.group(1),
                    'timestamp': time.time()
                })
    
    # Save parsed data
    data = {
        'positions': positions,
        'timestamps': timestamps,
        'modes': modes,
        'num_samples': len(positions)
    }
    
    with open(f"{output_dir}/follower_data.json", "w") as f:
        json.dump(data, f, indent=2)
    
    print(f"[simulate_with_diagnostics] Parsed {len(positions)} position samples")

def generate_diagnostic_plots(output_dir):
    """Generate diagnostic plots using matplotlib"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
        import json
        from datetime import datetime
        
        # Load data
        with open(f"{output_dir}/follower_data.json", "r") as f:
            data = json.load(f)
        
        positions = np.array(data['positions'])
        timestamps = np.array(data['timestamps'])
        
        if len(positions) == 0:
            print("[simulate_with_diagnostics] No position data to plot")
            return
        
        # Convert timestamps to relative time in seconds
        t_start = timestamps[0]
        t_rel = (timestamps - t_start) / 1e6  # Convert microseconds to seconds
        
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 10))
        fig.suptitle(f'BILK Simulation Diagnostics - {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}', fontsize=16)
        
        # Plot 1: Joint positions over time
        ax1 = plt.subplot(2, 2, 1)
        joint_names = ['J1', 'J2', 'J3', 'J4']
        colors = ['blue', 'red', 'green', 'orange']
        
        for i in range(4):
            ax1.plot(t_rel, positions[:, i], color=colors[i], label=joint_names[i], linewidth=1.5)
        
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Joint Position (rad)')
        ax1.set_title('Joint Positions Over Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: 3D trajectory (first 3 joints)
        ax2 = plt.subplot(2, 2, 2, projection='3d')
        ax2.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2, alpha=0.7)
        ax2.scatter(positions[0, 0], positions[0, 1], positions[0, 2], color='green', s=100, label='Start')
        ax2.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], color='red', s=100, label='End')
        ax2.set_xlabel('J1 (rad)')
        ax2.set_ylabel('J2 (rad)')
        ax2.set_zlabel('J3 (rad)')
        ax2.set_title('3D Trajectory (J1-J3)')
        ax2.legend()
        
        # Plot 3: Latency analysis
        ax3 = plt.subplot(2, 2, 3)
        if len(t_rel) > 1:
            dt = np.diff(t_rel)
            expected_dt = 1.0 / 200  # Expected 200 Hz
            latency = (dt - expected_dt) * 1000  # Convert to ms
            
            ax3.plot(t_rel[1:], latency, 'purple', linewidth=1)
            ax3.axhline(y=0, color='black', linestyle='--', alpha=0.5)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Latency (ms)')
            ax3.set_title('Timing Latency (vs 200Hz expected)')
            ax3.grid(True, alpha=0.3)
            
            # Add statistics
            mean_latency = np.mean(latency)
            std_latency = np.std(latency)
            ax3.text(0.02, 0.98, f'Mean: {mean_latency:.2f}ms\nStd: {std_latency:.2f}ms', 
                    transform=ax3.transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Plot 4: Joint velocity
        ax4 = plt.subplot(2, 2, 4)
        if len(positions) > 1:
            velocities = np.diff(positions, axis=0) / np.diff(t_rel).reshape(-1, 1)
            
            for i in range(4):
                ax4.plot(t_rel[1:], velocities[:, i], color=colors[i], label=f'{joint_names[i]} vel', linewidth=1.5)
        
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Joint Velocity (rad/s)')
        ax4.set_title('Joint Velocities')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        plot_file = f"{output_dir}/diagnostic_plots.png"
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        print(f"[simulate_with_diagnostics] Saved diagnostic plots to {plot_file}")
        
        # Also save as PDF for better quality
        pdf_file = f"{output_dir}/diagnostic_plots.pdf"
        plt.savefig(pdf_file, bbox_inches='tight')
        print(f"[simulate_with_diagnostics] Saved diagnostic plots to {pdf_file}")
        
        plt.close()
        
        # Generate summary statistics
        generate_summary_stats(data, output_dir)
        
    except ImportError:
        print("[simulate_with_diagnostics] matplotlib not available. Install with: pip install matplotlib")
    except Exception as e:
        print(f"[simulate_with_diagnostics] Error generating plots: {e}")

def generate_summary_stats(data, output_dir):
    """Generate summary statistics"""
    import numpy as np
    
    positions = np.array(data['positions'])
    timestamps = np.array(data['timestamps'])
    
    if len(positions) == 0:
        return
    
    # Calculate statistics
    t_rel = (timestamps - timestamps[0]) / 1e6
    dt = np.diff(t_rel)
    
    stats = {
        'duration_seconds': t_rel[-1] if len(t_rel) > 0 else 0,
        'total_samples': len(positions),
        'average_frequency_hz': len(positions) / t_rel[-1] if t_rel[-1] > 0 else 0,
        'timing_stats': {
            'mean_interval_ms': np.mean(dt) * 1000 if len(dt) > 0 else 0,
            'std_interval_ms': np.std(dt) * 1000 if len(dt) > 0 else 0,
            'min_interval_ms': np.min(dt) * 1000 if len(dt) > 0 else 0,
            'max_interval_ms': np.max(dt) * 1000 if len(dt) > 0 else 0
        },
        'joint_ranges': {
            f'J{i+1}': {
                'min': float(np.min(positions[:, i])),
                'max': float(np.max(positions[:, i])),
                'range': float(np.max(positions[:, i]) - np.min(positions[:, i]))
            } for i in range(4)
        }
    }
    
    # Save statistics
    with open(f"{output_dir}/summary_stats.json", "w") as f:
        json.dump(stats, f, indent=2)
    
    # Print summary
    print(f"\n[simulate_with_diagnostics] === SIMULATION SUMMARY ===")
    print(f"Duration: {stats['duration_seconds']:.2f} seconds")
    print(f"Total samples: {stats['total_samples']}")
    print(f"Average frequency: {stats['average_frequency_hz']:.1f} Hz")
    print(f"Timing - Mean: {stats['timing_stats']['mean_interval_ms']:.2f}ms, Std: {stats['timing_stats']['std_interval_ms']:.2f}ms")
    print(f"Joint ranges:")
    for i in range(4):
        j_stats = stats['joint_ranges'][f'J{i+1}']
        print(f"  J{i+1}: {j_stats['min']:.3f} to {j_stats['max']:.3f} (range: {j_stats['range']:.3f})")
    print(f"=====================================\n")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Run BILK simulation with diagnostics')
    parser.add_argument('--duration', type=int, default=30, help='Simulation duration in seconds (default: 30)')
    parser.add_argument('--output-dir', type=str, help='Output directory (default: auto-generated)')
    
    args = parser.parse_args()
    
    if args.output_dir:
        os.makedirs(args.output_dir, exist_ok=True)
    
    run_simulation_with_diagnostics(args.duration)
