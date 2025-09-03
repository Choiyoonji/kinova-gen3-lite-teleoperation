import numpy as np
from typing import Tuple, List, Optional

#!/usr/bin/env python3



class FifthOrderTrajectoryPlanner:
    """
    Fifth-order polynomial trajectory planner that considers lookahead time,
    current velocity, and acceleration to generate smooth trajectories.
    """
    
    def __init__(self):
        """Initialize the trajectory planner."""
        pass
        
    def plan_trajectory(self, 
                        start_pos: np.ndarray,
                        target_pos: np.ndarray,
                        current_vel: np.ndarray,
                        current_acc: np.ndarray,
                        lookahead_time: float,
                        target_vel: Optional[np.ndarray] = None,
                        target_acc: Optional[np.ndarray] = None) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        Generate a fifth-order polynomial trajectory.
        
        Args:
            start_pos: Starting position (current position)
            target_pos: Target position to reach
            current_vel: Current velocity
            current_acc: Current acceleration
            lookahead_time: Time to reach the target
            target_vel: Target velocity (default: zero)
            target_acc: Target acceleration (default: zero)
            
        Returns:
            time_points: Array of time points
            trajectory_points: List of [position, velocity, acceleration] at each time point
        """
        # Ensure inputs are numpy arrays
        start_pos = np.array(start_pos)
        target_pos = np.array(target_pos)
        current_vel = np.array(current_vel)
        current_acc = np.array(current_acc)
        
        # Set default target velocity and acceleration if not provided
        if target_vel is None:
            target_vel = np.zeros_like(start_pos)
        if target_acc is None:
            target_acc = np.zeros_like(start_pos)
            
        # Boundary conditions
        p0 = start_pos
        v0 = current_vel
        a0 = current_acc
        pf = target_pos
        vf = target_vel
        af = target_acc
        T = lookahead_time
        
        # Calculate quintic polynomial coefficients
        A = np.zeros((6, len(p0)))
        
        for i in range(len(p0)):
            # Solve for the coefficients of the quintic polynomial
            # x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            
            # System of equations based on boundary conditions
            # p0 = a0
            # v0 = a1
            # a0 = 2*a2
            # pf = a0 + a1*T + a2*T^2 + a3*T^3 + a4*T^4 + a5*T^5
            # vf = a1 + 2*a2*T + 3*a3*T^2 + 4*a4*T^3 + 5*a5*T^4
            # af = 2*a2 + 6*a3*T + 12*a4*T^2 + 20*a5*T^3
            
            a0 = p0[i]
            a1 = v0[i]
            a2 = a0 / 2.0
            
            # Matrix to solve for a3, a4, a5
            A_matrix = np.array([
                [T**3, T**4, T**5],
                [3*T**2, 4*T**3, 5*T**4],
                [6*T, 12*T**2, 20*T**3]
            ])
            
            b = np.array([
                pf[i] - a0 - a1*T - a2*T**2,
                vf[i] - a1 - 2*a2*T,
                af[i] - 2*a2
            ])
            
            x = np.linalg.solve(A_matrix, b)
            a3, a4, a5 = x
            
            A[:, i] = [a0, a1, a2, a3, a4, a5]
        
        # Generate trajectory points
        num_points = 100
        time_points = np.linspace(0, T, num_points)
        trajectory_points = []
        
        for t in time_points:
            # Position: a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            pos = A[0, :] + A[1, :]*t + A[2, :]*t**2 + A[3, :]*t**3 + A[4, :]*t**4 + A[5, :]*t**5
            
            # Velocity: a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
            vel = A[1, :] + 2*A[2, :]*t + 3*A[3, :]*t**2 + 4*A[4, :]*t**3 + 5*A[5, :]*t**4
            
            # Acceleration: 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
            acc = 2*A[2, :] + 6*A[3, :]*t + 12*A[4, :]*t**2 + 20*A[5, :]*t**3
            
            trajectory_points.append([pos, vel, acc])
        
        return time_points, trajectory_points
    
    def get_position_at_time(self, coeffs: np.ndarray, t: float) -> np.ndarray:
        """
        Calculate position at a specific time using the polynomial coefficients.
        
        Args:
            coeffs: 6xN array of polynomial coefficients
            t: Time at which to evaluate the polynomial
            
        Returns:
            Position at time t
        """
        return coeffs[0, :] + coeffs[1, :]*t + coeffs[2, :]*t**2 + coeffs[3, :]*t**3 + coeffs[4, :]*t**4 + coeffs[5, :]*t**5
    
    def get_velocity_at_time(self, coeffs: np.ndarray, t: float) -> np.ndarray:
        """
        Calculate velocity at a specific time using the polynomial coefficients.
        
        Args:
            coeffs: 6xN array of polynomial coefficients
            t: Time at which to evaluate the derivative
            
        Returns:
            Velocity at time t
        """
        return coeffs[1, :] + 2*coeffs[2, :]*t + 3*coeffs[3, :]*t**2 + 4*coeffs[4, :]*t**3 + 5*coeffs[5, :]*t**4
    
    def get_acceleration_at_time(self, coeffs: np.ndarray, t: float) -> np.ndarray:
        """
        Calculate acceleration at a specific time using the polynomial coefficients.
        
        Args:
            coeffs: 6xN array of polynomial coefficients
            t: Time at which to evaluate the second derivative
            
        Returns:
            Acceleration at time t
        """
        return 2*coeffs[2, :] + 6*coeffs[3, :]*t + 12*coeffs[4, :]*t**2 + 20*coeffs[5, :]*t**3


if __name__ == "__main__":
    # Example usage
    planner = FifthOrderTrajectoryPlanner()
    
    # Initial conditions
    start_pos = np.array([0.0, 0.0, 0.0])
    target_pos = np.array([1.0, 1.0, 0.5])
    current_vel = np.array([0.1, 0.0, 0.0])
    current_acc = np.array([0.0, 0.0, 0.0])
    lookahead_time = 2.0
    
    # Generate trajectory
    time_points, trajectory = planner.plan_trajectory(
        start_pos, target_pos, current_vel, current_acc, lookahead_time
    )
    
    # Print some trajectory points
    print(f"Start: pos={trajectory[0][0]}, vel={trajectory[0][1]}, acc={trajectory[0][2]}")
    print(f"Middle: pos={trajectory[len(trajectory)//2][0]}, vel={trajectory[len(trajectory)//2][1]}, acc={trajectory[len(trajectory)//2][2]}")
    print(f"End: pos={trajectory[-1][0]}, vel={trajectory[-1][1]}, acc={trajectory[-1][2]}")