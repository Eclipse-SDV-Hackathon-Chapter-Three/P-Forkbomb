//
// Copyright (c) 2025 The X-Verse <https://github.com/The-Xverse>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

use log::{info, debug};
use crate::uprotocol_handler::{LidarMeasurement, PointCoords};

#[derive(Debug, Clone)]
pub struct PIDResult {
    pub acceleration: f64,      // Keep for compatibility (m/s²)
    pub throttle: f64,          // 0.0 to 1.0 (0% to 100%)
    pub brake: f64,             // 0.0 to 1.0 (0% to 100%)
    pub emergency_brake_engaged: bool,
    pub emergency_reason: Option<String>,
    pub manual_brake_detected: bool,
    pub cruise_should_disengage: bool,
    pub cruise_can_reengage: bool,
}

impl PIDResult {
    pub fn new(acceleration: f64) -> Self {
        let (throttle, brake) = Self::acceleration_to_throttle_brake(acceleration);
        Self {
            acceleration,
            throttle,
            brake,
            emergency_brake_engaged: false,
            emergency_reason: None,
            manual_brake_detected: false,
            cruise_should_disengage: false,
            cruise_can_reengage: false,
        }
    }
    
    pub fn emergency(acceleration: f64, reason: String) -> Self {
        let (throttle, brake) = Self::acceleration_to_throttle_brake(acceleration);
        Self {
            acceleration,
            throttle,
            brake,
            emergency_brake_engaged: true,
            emergency_reason: Some(reason),
            manual_brake_detected: false,
            cruise_should_disengage: true,
            cruise_can_reengage: false,
        }
    }
    
    pub fn manual_brake(acceleration: f64) -> Self {
        let (throttle, brake) = Self::acceleration_to_throttle_brake(acceleration);
        Self {
            acceleration,
            throttle,
            brake,
            emergency_brake_engaged: false,
            emergency_reason: None,
            manual_brake_detected: true,
            cruise_should_disengage: true,
            cruise_can_reengage: false,
        }
    }
    
    pub fn with_reengage_capability(mut self) -> Self {
        self.cruise_can_reengage = true;
        self
    }
    
    /// Convert acceleration (m/s²) to throttle/brake values (0.0-1.0)
    /// Uses smart scaling based on speed error for cruise control
    fn acceleration_to_throttle_brake(acceleration: f64) -> (f64, f64) {
        if acceleration > 0.0 {
            // Positive acceleration -> throttle
            // Use progressive scaling: small accelerations get small throttle
            let throttle = if acceleration <= 0.5 {
                // For gentle acceleration (0-0.5 m/s²), use 0-20% throttle
                acceleration * 0.4  // 0.5 * 0.4 = 0.2 (20%)
            } else if acceleration <= 1.5 {
                // For moderate acceleration (0.5-1.5 m/s²), use 20-60% throttle
                0.2 + (acceleration - 0.5) * 0.4  // 20% + up to 40% more
            } else {
                // For higher acceleration (1.5+ m/s²), use 60-100% throttle
                0.6 + (acceleration - 1.5) * 0.267  // 60% + remaining to 100%
            };
            (throttle.min(1.0).max(0.0), 0.0)
        } else {
            // Negative acceleration -> brake
            let abs_decel = -acceleration;
            let brake = if abs_decel <= 0.5 {
                // For gentle braking (0-0.5 m/s²), use 0-15% brake
                abs_decel * 0.3  // 0.5 * 0.3 = 0.15 (15%)
            } else if abs_decel <= 2.0 {
                // For moderate braking (0.5-2.0 m/s²), use 15-50% brake
                0.15 + (abs_decel - 0.5) * 0.233  // 15% + up to 35% more
            } else {
                // For hard braking (2.0+ m/s²), use 50-100% brake
                0.5 + (abs_decel - 2.0) * 0.083  // 50% + remaining to 100%
            };
            (0.0, brake.min(1.0).max(0.0))
        }
    }
}

pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,
    velocity_error: f64,
    previous_error: f64,
    accumulated_error: f64,
    previous_time: f64,
    // Emergency brake configuration
    emergency_stop_distance: f64,
    slow_down_distance: f64,
    max_braking_acceleration: f64,
    // Manual brake detection
    previous_velocity: f64,
    manual_brake_threshold: f64, // Deceleration threshold to detect manual braking
    cruise_suspended: bool,      // Track if cruise control is temporarily suspended
    target_speed_tolerance: f64, // How close to target speed before re-engaging
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self::new_with_emergency_config(kp, ki, kd, 3.0, 15.0, -10.0)
    }

    pub fn new_with_emergency_config(
        kp: f64, 
        ki: f64, 
        kd: f64, 
        emergency_stop_distance: f64,
        slow_down_distance: f64,
        max_braking_acceleration: f64
    ) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            velocity_error: 0.0,
            previous_error: 0.0,
            accumulated_error: 0.0,
            previous_time: 0.0,
            emergency_stop_distance,
            slow_down_distance,
            max_braking_acceleration,
            previous_velocity: 0.0,
            manual_brake_threshold: -2.0, // Detect manual braking at -2 m/s² or more
            cruise_suspended: false,
            target_speed_tolerance: 2.0,   // Re-engage when within 2 m/s of target
        }
    }

    /// Configure emergency brake parameters
    pub fn set_emergency_config(&mut self, emergency_stop_distance: f64, slow_down_distance: f64, max_braking_acceleration: f64) {
        self.emergency_stop_distance = emergency_stop_distance;
        self.slow_down_distance = slow_down_distance;
        self.max_braking_acceleration = max_braking_acceleration;
    }

    /// Get current emergency brake configuration
    pub fn get_emergency_config(&self) -> (f64, f64, f64) {
        (self.emergency_stop_distance, self.slow_down_distance, self.max_braking_acceleration)
    }

    pub fn set_manual_brake_config(&mut self, brake_threshold: f64, speed_tolerance: f64) {
        self.manual_brake_threshold = brake_threshold;
        self.target_speed_tolerance = speed_tolerance;
    }

    /// Get manual brake configuration
    pub fn get_manual_brake_config(&self) -> (f64, f64, bool) {
        (self.manual_brake_threshold, self.target_speed_tolerance, self.cruise_suspended)
    }

    /// Force cruise control suspension (for testing)
    pub fn suspend_cruise_control(&mut self) {
        self.cruise_suspended = true;
        info!("Cruise control manually suspended");
    }

    /// Check if cruise control is currently suspended
    pub fn is_cruise_suspended(&self) -> bool {
        self.cruise_suspended
    }

    pub fn compute(
        &mut self, 
        desired_velocity: f64, 
        current_velocity: f64, 
        current_time: f64, 
        lidar_data: Option<&LidarMeasurement>,
        throttle_input: f64,  // 0.0-1.0 from driver/control system
        steer_input: f64,     // 0.0-1.0 steering amount
        brake_input: f64      // 0.0-1.0 from driver/control system
    ) -> Result<PIDResult, String> {
        if self.previous_time == 0.0 {
            self.previous_time = current_time;
            self.previous_velocity = current_velocity;
            return Ok(PIDResult::new(0.0));
        }

        let delta_time = current_time - self.previous_time;
        self.previous_time = current_time;
        
        // Detect manual braking by analyzing velocity change
        let velocity_change = current_velocity - self.previous_velocity;
        let actual_acceleration = if delta_time > 0.0 { velocity_change / delta_time } else { 0.0 };
        
        // Check for manual braking using actual brake input
        const BRAKE_THRESHOLD: f64 = 0.1; // 10% brake input triggers manual brake detection
        let manual_brake_detected = brake_input > BRAKE_THRESHOLD;
        
        if manual_brake_detected {
            info!("MANUAL BRAKE DETECTED: Brake input {:.1}% detected, suspending cruise control", brake_input * 100.0);
            self.cruise_suspended = true;
            self.previous_velocity = current_velocity;
            return Ok(PIDResult::manual_brake(-brake_input * 3.0)); // Convert brake % to deceleration
        }
        
        // Check if cruise control can be re-engaged
        let speed_difference = (desired_velocity - current_velocity).abs();
        let can_reengage = self.cruise_suspended && 
                          speed_difference <= self.target_speed_tolerance &&
                          current_velocity > 0.0 && // Must be moving
                          actual_acceleration >= -0.5; // Not braking hard
        
        if can_reengage {
            info!("CRUISE CONTROL RE-ENGAGEMENT: Speed difference {:.1} m/s is within tolerance {:.1} m/s", 
                  speed_difference, self.target_speed_tolerance);
            self.cruise_suspended = false;
        }
        
        if self.cruise_suspended {
            self.previous_velocity = current_velocity;
            let result = PIDResult::new(0.0); // No PID intervention
            return Ok(if can_reengage { result.with_reengage_capability() } else { result });
        }

        // Apply steering compensation - reduce desired speed when turning
        let steering_factor = Self::calculate_steering_compensation(steer_input);
        let adjusted_desired_velocity = desired_velocity * steering_factor;
        
        if steering_factor < 1.0 {
            let direction = if steer_input > 0.0 { "right" } else { "left" };
            info!("STEERING COMPENSATION: Reducing target speed from {:.1} to {:.1} m/s due to {:.1}% {} steering", 
                  desired_velocity, adjusted_desired_velocity, steer_input.abs() * 100.0, direction);
        }

        // Check for obstacles using lidar data and print closest position
        let mut modified_desired_velocity = adjusted_desired_velocity;
        if let Some(lidar) = lidar_data {
            if !lidar.is_empty && !lidar.detections.is_empty() {
                // Find the closest detection in the vehicle's path
                let mut closest_distance = f64::MAX;
                let mut closest_position: Option<&PointCoords> = None;
                
                // Define vehicle path constraints
                const PATH_WIDTH: f64 = 3.0; // meters (lane width with some margin)
                const MIN_HEIGHT: f64 = 0.3; // meters (ignore ground-level objects)
                const MAX_HEIGHT: f64 = 2.5; // meters (ignore overhead objects)
                const MAX_RANGE: f64 = 30.0; // meters (reasonable detection range)
                
                for detection in &lidar.detections {
                    let point = &detection.point;
                
                    if point.x > 1.0 && point.x < MAX_RANGE &&  // In front, with 1m minimum
                       point.y.abs() < PATH_WIDTH / 2.0 &&      // Within lane width
                       point.z > MIN_HEIGHT && point.z < MAX_HEIGHT { // At vehicle height
                        
                        // Use only forward distance for path-blocking obstacles
                        let forward_distance = point.x;
                        
                        if forward_distance < closest_distance {
                            closest_distance = forward_distance;
                            closest_position = Some(point);
                        }
                    }
                }
                
                if let Some(pos) = closest_position {
                    info!("LIDAR: Closest obstacle in vehicle path at position: x={:.2}m, y={:.2}m, z={:.2}m, forward_distance={:.2}m", 
                          pos.x, pos.y, pos.z, closest_distance);
                    
                    // Calculate velocity-dependent safety distances
                    let velocity_factor = (current_velocity / 10.0).max(1.0); // Scale with velocity, min factor of 1
                    let dynamic_emergency_distance = self.emergency_stop_distance * velocity_factor;
                    let dynamic_slow_down_distance = self.slow_down_distance * velocity_factor;
                    
                    if closest_distance < dynamic_emergency_distance {
                        info!("EMERGENCY BRAKE: Obstacle in vehicle path at {:.2}m forward distance! (threshold: {:.2}m)", 
                              closest_distance, dynamic_emergency_distance);
                        
                        // Calculate emergency brake intensity based on distance and velocity
                        let urgency_factor = 1.0 - (closest_distance / dynamic_emergency_distance);
                        let emergency_acceleration = self.max_braking_acceleration * urgency_factor.max(0.5);
                        
                        let reason = format!("Obstacle detected at {:.1}m (emergency threshold: {:.1}m)", 
                                            closest_distance, dynamic_emergency_distance);
                        
                        let result = PIDResult::emergency(emergency_acceleration, reason);
                        info!("EMERGENCY BRAKE: Applying {:.2} m/s² braking (brake: {:.1}%) - CRUISE CONTROL WILL BE DISENGAGED", 
                              emergency_acceleration, result.brake * 100.0);
                        return Ok(result);
                    } else if closest_distance < dynamic_slow_down_distance {
                        // Gradual speed reduction with distance-based intensity
                        let distance_factor = (closest_distance - dynamic_emergency_distance) / 
                                             (dynamic_slow_down_distance - dynamic_emergency_distance);
                        let brake_intensity = 1.0 - distance_factor;
                        
                        // Apply both speed reduction and gentle braking
                        modified_desired_velocity = desired_velocity * distance_factor.max(0.2); // Don't go below 20% of desired speed
                        
                        info!("COLLISION AVOIDANCE: Reducing speed to {:.2} m/s due to obstacle at {:.2}m forward distance (threshold: {:.2}m)", 
                              modified_desired_velocity, closest_distance, dynamic_slow_down_distance);
                        
                        // If we need aggressive slowing, apply immediate gentle braking
                        if brake_intensity > 0.5 {
                            let gentle_brake = self.max_braking_acceleration * 0.3 * brake_intensity;
                            let result = PIDResult::new(gentle_brake.max(-1.0));
                            info!("COLLISION AVOIDANCE: Applying gentle braking {:.2} m/s² (brake: {:.1}%)", 
                                  gentle_brake, result.brake * 100.0);
                            return Ok(result);
                        }
                    }
                }
            }
        }

        if delta_time <= 0.0 {
            if delta_time < -0.001 {
                return Err(format!("Significant negative delta_time: {:.6} seconds. current_time={:.6}, previous_time={:.6}", 
                                 delta_time, current_time, self.previous_time));
            } else {
                let result = self.compute_pid(modified_desired_velocity, current_velocity, 0.001)?;
                self.previous_velocity = current_velocity;
                return Ok(result);
            }
        }

        let result = self.compute_pid(modified_desired_velocity, current_velocity, delta_time)?;
        self.previous_velocity = current_velocity;
        Ok(result)
    }

    fn compute_pid(&mut self, desired_velocity: f64, current_velocity: f64, delta_time: f64) -> Result<PIDResult, String> {
        // Check if we're significantly over the desired speed (more than 15% overspeed)
        if current_velocity > desired_velocity + (desired_velocity * 0.15) {
            // Apply gentle negative acceleration (braking) when we need to slow down
            let speed_excess = current_velocity - desired_velocity;
            
            // Use a much gentler braking approach
            let gentle_braking = if speed_excess > 2.0 {
                -1.0  // Maximum gentle braking for significant overspeed
            } else {
                -speed_excess * 0.8  // Proportional gentle braking
            };
            let result = PIDResult::new(gentle_braking);
            info!("SPEED CONTROL: Applying gentle braking {:.2} m/s² (brake: {:.1}%) for speed excess {:.1} m/s", 
                  gentle_braking, result.brake * 100.0, speed_excess);
            return Ok(result);
        }
        
        // Normal PID control for acceleration and gentle deceleration
        self.previous_error = self.velocity_error;
        self.velocity_error = desired_velocity - current_velocity;
        self.accumulated_error += self.velocity_error * delta_time;
        let derivative_error = (self.velocity_error - self.previous_error) / delta_time;
        let acceleration = (self.kp * self.velocity_error)
            + (self.ki * self.accumulated_error)
            + (self.kd * derivative_error);
        
        // Limit acceleration to gentler values
        let limited_acceleration = acceleration.max(-1.5).min(1.5); // Much gentler limits: -1.5 to +3 m/s²
        let result = PIDResult::new(limited_acceleration);
        
        if limited_acceleration > 0.0 {
            debug!("PID CONTROL: Throttle {:.1}% ({:.2} m/s²)", result.throttle * 100.0, limited_acceleration);
        } else if limited_acceleration < 0.0 {
            debug!("PID CONTROL: Brake {:.1}% ({:.2} m/s²)", result.brake * 100.0, limited_acceleration);
        }
        
        Ok(result)
    }

    /// Calculate speed reduction factor based on steering input
    /// More steering = more speed reduction for safer cornering
    /// steer_input: -1.0 (full left) to 1.0 (full right)
    fn calculate_steering_compensation(steer_input: f64) -> f64 {
        // Use absolute value since turning left or right both require speed reduction
        let abs_steering = steer_input.abs();
        
        const MAX_SPEED_REDUCTION: f64 = 0.8; // Maximum 20% speed reduction at full steering
        const STEERING_SENSITIVITY: f64 = 0.3; // Start reducing at 30% steering (0.3 abs value)
        
        if abs_steering <= STEERING_SENSITIVITY {
            1.0 // No speed reduction for gentle steering
        } else {
            // Progressive speed reduction: 30% steering = 100% speed, 100% steering = 80% speed
            let reduction_factor = (abs_steering - STEERING_SENSITIVITY) / (1.0 - STEERING_SENSITIVITY);
            1.0 - (reduction_factor * (1.0 - MAX_SPEED_REDUCTION))
        }
    }

    pub fn reset(&mut self) {
        self.velocity_error = 0.0;
        self.previous_error = 0.0;
        self.accumulated_error = 0.0;
        self.previous_time = 0.0;
        self.previous_velocity = 0.0;
        self.cruise_suspended = false;
    }
}