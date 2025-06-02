#!/usr/bin/env python3
"""
Drone PID Tuning Simulation
Helps determine optimal Ki and Kd values for pitch, roll, and yaw control
Using existing Kp values from the flight controller
"""

import numpy as np
import matplotlib.pyplot as plt
import json

class DronePIDSimulator:
    def __init__(self):
        # Physical parameters (simplified quadcopter model)
        self.mass = 1.2  # kg (typical small drone)
        self.arm_length = 0.225  # meters (distance from center to motor)
        self.g = 9.81  # gravity
        
        # Moments of inertia (kg*m^2)
        self.Ixx = 0.0086  # roll axis
        self.Iyy = 0.0086  # pitch axis  
        self.Izz = 0.0172  # yaw axis
        
        # Motor response time constant (seconds)
        self.motor_tau = 0.02
        
        # PID gains from your controller
        self.Kp_pitch = 0.025
        self.Kp_roll = 0.025
        self.Kp_yaw = 0.015
        
        # To be tuned
        self.Ki_pitch = 0.0
        self.Kd_pitch = 0.0
        self.Ki_roll = 0.0
        self.Kd_roll = 0.0
        self.Ki_yaw = 0.0
        self.Kd_yaw = 0.0
        
        # PID state
        self.reset_pid_state()
        
        # Simulation time step
        self.dt = 0.001  # 1ms (1000Hz)
        
    def reset_pid_state(self):
        """Reset PID controller states"""
        self.pitch_integral = 0.0
        self.roll_integral = 0.0
        self.yaw_integral = 0.0
        self.pitch_prev_error = 0.0
        self.roll_prev_error = 0.0
        self.yaw_prev_error = 0.0
        
    def pid_control(self, error, integral, prev_error, Kp, Ki, Kd, max_integral=50.0):
        """Generic PID calculation matching your Arduino implementation"""
        # Update integral with anti-windup
        integral += error * self.dt
        integral = np.clip(integral, -max_integral, max_integral)
        
        # Calculate derivative
        if self.dt > 0:
            derivative = (error - prev_error) / self.dt
        else:
            derivative = 0
            
        # PID output
        output = Kp * error + Ki * integral + Kd * derivative
        
        return output, integral
        
    def simulate_step_response(self, axis, target_angle, duration=3.0, 
                             Ki_values=[0.0], Kd_values=[0.0]):
        """Simulate step response for a single axis"""
        results = []
        
        for Ki in Ki_values:
            for Kd in Kd_values:
                # Set gains for this test
                if axis == 'pitch':
                    self.Ki_pitch = Ki
                    self.Kd_pitch = Kd
                    Kp = self.Kp_pitch
                    I = self.Iyy
                elif axis == 'roll':
                    self.Ki_roll = Ki
                    self.Kd_roll = Kd
                    Kp = self.Kp_roll
                    I = self.Ixx
                elif axis == 'yaw':
                    self.Ki_yaw = Ki
                    self.Kd_yaw = Kd
                    Kp = self.Kp_yaw
                    I = self.Izz
                    
                # Reset states
                self.reset_pid_state()
                angle = 0.0
                angular_vel = 0.0
                integral = 0.0
                prev_error = 0.0
                
                # Data storage
                time_data = []
                angle_data = []
                error_data = []
                control_data = []
                
                # Simulation loop
                steps = int(duration / self.dt)
                for i in range(steps):
                    t = i * self.dt
                    
                    # Calculate error
                    error = target_angle - angle
                    
                    # PID control
                    if axis == 'pitch':
                        control_output, self.pitch_integral = self.pid_control(
                            error, self.pitch_integral, self.pitch_prev_error,
                            self.Kp_pitch, self.Ki_pitch, self.Kd_pitch)
                        self.pitch_prev_error = error
                    elif axis == 'roll':
                        control_output, self.roll_integral = self.pid_control(
                            error, self.roll_integral, self.roll_prev_error,
                            self.Kp_roll, self.Ki_roll, self.Kd_roll)
                        self.roll_prev_error = error
                    elif axis == 'yaw':
                        control_output, self.yaw_integral = self.pid_control(
                            error, self.yaw_integral, self.yaw_prev_error,
                            self.Kp_yaw, self.Ki_yaw, self.Kd_yaw)
                        self.yaw_prev_error = error
                    
                    # Limit control output (matching Arduino's motor limits)
                    control_output = np.clip(control_output, -0.1, 0.1)
                    
                    # Simple physics: torque = control_output * some_scaling
                    # Angular acceleration = torque / I
                    torque = control_output * 10.0  # Scaling factor
                    angular_accel = torque / I
                    
                    # Update states
                    angular_vel += angular_accel * self.dt
                    angular_vel *= 0.98  # Air resistance damping
                    angle += angular_vel * self.dt
                    
                    # Store data
                    time_data.append(t)
                    angle_data.append(angle)
                    error_data.append(error)
                    control_data.append(control_output)
                
                # Calculate performance metrics
                steady_state_error = abs(angle_data[-1] - target_angle)
                
                # Find rise time (10% to 90%)
                target_10 = 0.1 * target_angle
                target_90 = 0.9 * target_angle
                rise_start = next((i for i, a in enumerate(angle_data) if a >= target_10), None)
                rise_end = next((i for i, a in enumerate(angle_data) if a >= target_90), None)
                rise_time = (rise_end - rise_start) * self.dt if rise_start and rise_end else None
                
                # Find overshoot
                max_angle = max(angle_data) if target_angle > 0 else min(angle_data)
                overshoot = (max_angle - target_angle) / target_angle * 100 if target_angle != 0 else 0
                
                results.append({
                    'Ki': Ki,
                    'Kd': Kd,
                    'time': time_data,
                    'angle': angle_data,
                    'error': error_data,
                    'control': control_data,
                    'steady_state_error': steady_state_error,
                    'rise_time': rise_time,
                    'overshoot': overshoot
                })
                
        return results
        
    def plot_step_responses(self, axis, results, target_angle):
        """Plot multiple step responses for comparison"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f'{axis.capitalize()} Step Response - Target: {target_angle}°')
        
        # Different line styles for different responses
        styles = ['-', '--', '-.', ':']
        colors = plt.cm.viridis(np.linspace(0, 1, len(results)))
        
        for i, result in enumerate(results):
            label = f"Ki={result['Ki']:.4f}, Kd={result['Kd']:.4f}"
            style = styles[i % len(styles)]
            color = colors[i]
            
            # Angle response
            ax1.plot(result['time'], result['angle'], style, color=color, label=label)
            ax1.axhline(y=target_angle, color='red', linestyle='--', alpha=0.5)
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Angle (degrees)')
            ax1.set_title('Angle Response')
            ax1.grid(True)
            ax1.legend()
            
            # Error over time
            ax2.plot(result['time'], result['error'], style, color=color, label=label)
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Error (degrees)')
            ax2.set_title('Tracking Error')
            ax2.grid(True)
            
            # Control output
            ax3.plot(result['time'], result['control'], style, color=color, label=label)
            ax3.set_xlabel('Time (s)')
            ax3.set_ylabel('Control Output')
            ax3.set_title('PID Output')
            ax3.grid(True)
            
        # Performance metrics table
        ax4.axis('tight')
        ax4.axis('off')
        
        table_data = [['Ki', 'Kd', 'Rise Time (s)', 'Overshoot (%)', 'SS Error (°)']]
        for result in results:
            rise_time_str = f"{result['rise_time']:.3f}" if result['rise_time'] else "N/A"
            table_data.append([
                f"{result['Ki']:.4f}",
                f"{result['Kd']:.4f}",
                rise_time_str,
                f"{result['overshoot']:.1f}",
                f"{result['steady_state_error']:.3f}"
            ])
            
        table = ax4.table(cellText=table_data, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2)
        ax4.set_title('Performance Metrics')
        
        plt.tight_layout()
        plt.savefig(f'{axis}_step_response_comparison.png', dpi=150)
        plt.close()
        print(f"Saved plot: {axis}_step_response_comparison.png")
        
    def find_optimal_gains(self, axis='pitch'):
        """Test multiple Ki and Kd combinations to find optimal gains"""
        print(f"\n=== Finding optimal gains for {axis} ===")
        print(f"Using Kp = {getattr(self, f'Kp_{axis}')}")
        
        # Test ranges - start conservative
        if axis == 'yaw':
            Ki_range = [0.0, 0.00005, 0.0001, 0.0002]
            Kd_range = [0.0, 0.0005, 0.001, 0.002]
        else:  # pitch/roll
            Ki_range = [0.0, 0.0001, 0.0002, 0.0004]
            Kd_range = [0.0, 0.001, 0.002, 0.004]
            
        # Target angle for testing
        target = 10.0 if axis != 'yaw' else 30.0
        
        best_score = float('inf')
        best_Ki = 0
        best_Kd = 0
        
        print("\nTesting combinations...")
        for Ki in Ki_range:
            for Kd in Kd_range:
                results = self.simulate_step_response(axis, target, 
                                                    duration=2.0, 
                                                    Ki_values=[Ki], 
                                                    Kd_values=[Kd])
                result = results[0]
                
                # Scoring function (lower is better)
                # Penalize overshoot, steady state error, and slow rise time
                overshoot_penalty = abs(result['overshoot']) * 0.1 if result['overshoot'] > 5 else 0
                rise_time_penalty = result['rise_time'] * 10 if result['rise_time'] else 100
                ss_error_penalty = result['steady_state_error'] * 100
                
                score = overshoot_penalty + rise_time_penalty + ss_error_penalty
                
                rise_time_str = f"{result['rise_time']:.3f}" if result['rise_time'] else "N/A"
                print(f"Ki={Ki:.4f}, Kd={Kd:.4f}: Score={score:.2f} "
                      f"(OS={result['overshoot']:.1f}%, RT={rise_time_str}s, "
                      f"SSE={result['steady_state_error']:.3f}°)")
                
                if score < best_score:
                    best_score = score
                    best_Ki = Ki
                    best_Kd = Kd
                    
        print(f"\nBest gains for {axis}: Ki={best_Ki:.4f}, Kd={best_Kd:.4f}")
        
        # Show comparison of P-only vs PID
        comparison_results = self.simulate_step_response(
            axis, target, duration=3.0,
            Ki_values=[0.0, best_Ki],
            Kd_values=[0.0, best_Kd]
        )
        
        # Filter to show P-only and best PID
        filtered_results = [
            comparison_results[0],  # P-only (Ki=0, Kd=0)
            comparison_results[-1]  # Best PID
        ]
        
        self.plot_step_responses(axis, filtered_results, target)
        
        return best_Ki, best_Kd

def main():
    """Run the tuning simulation"""
    print("Drone PID Tuning Simulation")
    print("=" * 40)
    print("Current Proportional Gains:")
    print("  Pitch: Kp = 0.025")
    print("  Roll:  Kp = 0.025")  
    print("  Yaw:   Kp = 0.015")
    print("\nFinding optimal Ki and Kd values...")
    
    sim = DronePIDSimulator()
    
    # Find optimal gains for each axis
    optimal_gains = {}
    
    for axis in ['pitch', 'roll', 'yaw']:
        Ki_opt, Kd_opt = sim.find_optimal_gains(axis)
        optimal_gains[axis] = {'Ki': Ki_opt, 'Kd': Kd_opt}
        
    # Print summary
    print("\n" + "=" * 40)
    print("RECOMMENDED PID GAINS:")
    print("=" * 40)
    print("\nPitch:")
    print(f"  Kp = {sim.Kp_pitch}")
    print(f"  Ki = {optimal_gains['pitch']['Ki']:.4f}")
    print(f"  Kd = {optimal_gains['pitch']['Kd']:.4f}")
    print("\nRoll:")
    print(f"  Kp = {sim.Kp_roll}")
    print(f"  Ki = {optimal_gains['roll']['Ki']:.4f}")
    print(f"  Kd = {optimal_gains['roll']['Kd']:.4f}")
    print("\nYaw:")
    print(f"  Kp = {sim.Kp_yaw}")
    print(f"  Ki = {optimal_gains['yaw']['Ki']:.4f}")
    print(f"  Kd = {optimal_gains['yaw']['Kd']:.4f}")
    
    # Save results
    with open('pid_tuning_results.json', 'w') as f:
        json.dump({
            'pitch': {'Kp': sim.Kp_pitch, **optimal_gains['pitch']},
            'roll': {'Kp': sim.Kp_roll, **optimal_gains['roll']},
            'yaw': {'Kp': sim.Kp_yaw, **optimal_gains['yaw']}
        }, f, indent=2)
    print("\nResults saved to pid_tuning_results.json")
    
    # Test disturbance rejection
    print("\n" + "=" * 40)
    print("Testing disturbance rejection...")
    
    # Set optimal gains
    sim.Ki_pitch = optimal_gains['pitch']['Ki']
    sim.Kd_pitch = optimal_gains['pitch']['Kd']
    
    # Simulate with disturbance
    sim.reset_pid_state()
    time_data = []
    angle_data = []
    disturbance_data = []
    
    angle = 0.0
    angular_vel = 0.0
    target = 0.0  # Try to maintain level
    
    for i in range(int(5.0 / sim.dt)):
        t = i * sim.dt
        
        # Add disturbance pulse at t=1s
        if 1.0 < t < 1.5:
            disturbance = 0.05  # External torque
        else:
            disturbance = 0.0
            
        error = target - angle
        control_output, sim.pitch_integral = sim.pid_control(
            error, sim.pitch_integral, sim.pitch_prev_error,
            sim.Kp_pitch, sim.Ki_pitch, sim.Kd_pitch)
        sim.pitch_prev_error = error
        
        # Physics with disturbance
        torque = control_output * 10.0 + disturbance
        angular_accel = torque / sim.Iyy
        angular_vel += angular_accel * sim.dt
        angular_vel *= 0.98
        angle += angular_vel * sim.dt
        
        time_data.append(t)
        angle_data.append(angle)
        disturbance_data.append(disturbance * 100)  # Scale for visibility
        
    # Plot disturbance rejection
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    fig.suptitle('Disturbance Rejection Test')
    
    ax1.plot(time_data, angle_data, 'b-', label='Pitch angle')
    ax1.axhline(y=0, color='red', linestyle='--', alpha=0.5, label='Target')
    ax1.set_ylabel('Angle (degrees)')
    ax1.legend()
    ax1.grid(True)
    
    ax2.plot(time_data, disturbance_data, 'r-', label='Disturbance')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Disturbance (scaled)')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig('disturbance_rejection_test.png', dpi=150)
    plt.close()
    print("Saved plot: disturbance_rejection_test.png")

if __name__ == "__main__":
    main() 