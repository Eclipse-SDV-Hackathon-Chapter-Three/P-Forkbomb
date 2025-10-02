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

use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};
use serde::{Deserialize, Serialize};
use serde_json;
use log::{info, debug, error, warn};
use up_rust::{UUri, UListener, UMessage, UMessageBuilder, UTransport, UPayloadFormat};
use up_transport_zenoh::UPTransportZenoh;


// New resource ID for control values
pub const RESOURCE_CONTROL_VALUES: u16 = 0x8004;

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ControlValues {
    pub throttle: f64,
    pub steer: f64,
    pub brake: f64,
}

use crate::pid_controller::PIDController;

#[derive(Debug, Serialize, Deserialize)]
struct VelocityStatus {
    velocity: f64,
}

#[derive(Debug, Serialize, Deserialize)]
struct ClockStatus {
    time: f64,
}

#[derive(Debug, Serialize, Deserialize)]
struct TargetSpeed {
    speed: f64,
}

#[derive(Debug, Serialize, Deserialize)]
struct EngageStatus {
    engaged: u8,
}

#[derive(Debug, Deserialize, Clone)]
pub struct LidarMeasurement {
    pub channel_count: u32,
    pub detections: Vec<LidarDetection>,
    pub horizontal_angle: f64,
    pub is_empty: bool,
    pub len: u32,
}

#[derive(Debug, Deserialize, Clone)]
pub struct LidarDetection {
    pub intensity: f64,
    pub point: PointCoords,
}

#[derive(Debug, Deserialize, Clone)]
pub struct PointCoords {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub struct UProtocolHandler {
    controller: Arc<Mutex<PIDController>>,
    transport: Arc<UPTransportZenoh>,
    
    // uProtocol URIs
    velocity_uri: UUri,
    clock_uri: UUri,
    engage_uri: UUri,
    target_speed_uri: UUri,
    actuation_uri: UUri,
    lidar_uri: UUri,
    control_values_uri: UUri,
    
    // State variables
    current_velocity: Arc<Mutex<f64>>,
    desired_velocity: Arc<Mutex<f64>>,
    current_time: Arc<Mutex<f64>>,
    previous_time: Arc<Mutex<f64>>,
    is_engaged: Arc<Mutex<u8>>,
    pid_active: Arc<Mutex<bool>>,
    latest_lidar_data: Arc<Mutex<Option<LidarMeasurement>>>,
    throttle: Arc<Mutex<f64>>,
    steer: Arc<Mutex<f64>>,
    brake: Arc<Mutex<f64>>,
    
    // Results storage
    results: Arc<Mutex<HashMap<String, Vec<f64>>>>,
}

impl UProtocolHandler {
    pub fn new(
        controller: PIDController,
        transport: UPTransportZenoh,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let mut results = HashMap::new();
        results.insert("desired_velocity".to_string(), Vec::new());
        results.insert("current_velocity".to_string(), Vec::new());
        results.insert("current_time".to_string(), Vec::new());
        results.insert("acceleration".to_string(), Vec::new());

        // Create URIs for different services
        let velocity_uri = UUri::try_from_parts("EGOVehicle", 0, 2, 0x8001)?;
        let clock_uri = UUri::try_from_parts("EGOVehicle", 0, 2, 0x8002)?;
        let engage_uri = UUri::try_from_parts("AAOS", 0, 2, 0x8002)?;
        let target_speed_uri = UUri::try_from_parts("AAOS", 0, 2, 0x8001)?;
        let actuation_uri = UUri::try_from_parts("CruiseControl", 0, 2, 0x8001)?;
        let lidar_uri = UUri::try_from_parts("EGOVehicle", 0, 2, 0x8003)?; // Use 0x8003 instead of 8003
        let control_values_uri = UUri::try_from_parts("CruiseControl", 0, 2, RESOURCE_CONTROL_VALUES)?;

        Ok(UProtocolHandler {
            controller: Arc::new(Mutex::new(controller)),
            transport: Arc::new(transport),
            velocity_uri,
            clock_uri,
            engage_uri,
            target_speed_uri,
            actuation_uri,
            lidar_uri,
            control_values_uri,
            current_velocity: Arc::new(Mutex::new(0.0)),
            desired_velocity: Arc::new(Mutex::new(0.0)),
            current_time: Arc::new(Mutex::new(0.0)),
            previous_time: Arc::new(Mutex::new(0.0)),
            is_engaged: Arc::new(Mutex::new(0)),
            pid_active: Arc::new(Mutex::new(false)),
            latest_lidar_data: Arc::new(Mutex::new(None)),
            throttle: Arc::new(Mutex::new(0.0)),
            steer: Arc::new(Mutex::new(0.0)),
            brake: Arc::new(Mutex::new(0.0)),
            results: Arc::new(Mutex::new(results)),
        })
    }

    pub async fn start(&self) -> Result<(), Box<dyn std::error::Error>> {
        info!("Starting UProtocolHandler subscribers...");

        // Register listeners for each subscription
        self.setup_clock_subscriber().await?;
        self.setup_velocity_subscriber().await?;
        self.setup_target_subscriber().await?;
        self.setup_engage_subscriber().await?;
        self.setup_lidar_subscriber().await?;
        self.setup_control_values_subscriber().await?;

        Ok(())
    }

    // Getter method to access the latest lidar data
    pub fn get_latest_lidar_data(&self) -> Option<LidarMeasurement> {
        let lidar_data = self.latest_lidar_data.lock().unwrap();
        lidar_data.clone()
    }
    
    // Helper method to get obstacle information from lidar data
    pub fn get_closest_obstacle(&self) -> Option<f64> {
        if let Some(ref measurement) = *self.latest_lidar_data.lock().unwrap() {
            if measurement.is_empty || measurement.detections.is_empty() {
                return None;
            }
            
            // Find the closest detection (minimum distance from origin)
            let mut min_distance = f64::MAX;
            
            for detection in &measurement.detections {
                let distance = (detection.point.x.powi(2) 
                              + detection.point.y.powi(2) 
                              + detection.point.z.powi(2)).sqrt();
                
                if distance < min_distance {
                    min_distance = distance;
                }
            }
            
            if min_distance < f64::MAX {
                Some(min_distance)
            } else {
                None
            }
        } else {
            None
        }
    }
    
    async fn setup_clock_subscriber(&self) -> Result<(), Box<dyn std::error::Error>> {
        let current_time_arc = Arc::clone(&self.current_time);
        let transport = Arc::clone(&self.transport);
        let clock_uri = self.clock_uri.clone();
        
        let listener = ClockListener::new(current_time_arc);
        transport.register_listener(&clock_uri, None, Arc::new(listener)).await?;
        
        info!("Timestamp subscriber registered");
        Ok(())
    }
    
    async fn setup_velocity_subscriber(&self) -> Result<(), Box<dyn std::error::Error>> {
        let current_velocity = Arc::clone(&self.current_velocity);
        let transport = Arc::clone(&self.transport);
        let velocity_uri = self.velocity_uri.clone();
        
        // Clone all necessary data for publish_acc
        let desired_velocity = Arc::clone(&self.desired_velocity);
        let current_time = Arc::clone(&self.current_time);
        let previous_time = Arc::clone(&self.previous_time);
        let pid_active = Arc::clone(&self.pid_active);
        let controller = Arc::clone(&self.controller);
        let results = Arc::clone(&self.results);
        let actuation_uri = self.actuation_uri.clone();
        let transport_for_publish = Arc::clone(&self.transport);
        
        let listener = VelocityListener::new(
            current_velocity,
            desired_velocity,
            current_time,
            previous_time,
            pid_active,
            controller,
            results,
            actuation_uri,
            transport_for_publish,
            Arc::clone(&self.latest_lidar_data),
            Arc::clone(&self.is_engaged),
            self.engage_uri.clone(),
            Arc::clone(&self.throttle),
            Arc::clone(&self.steer),
            Arc::clone(&self.brake),
        );
        
        transport.register_listener(&velocity_uri, None, Arc::new(listener)).await?;
        
        info!("Velocity subscriber registered");
        Ok(())
    }

    async fn setup_target_subscriber(&self) -> Result<(), Box<dyn std::error::Error>> {
        let desired_velocity = Arc::clone(&self.desired_velocity);
        let transport = Arc::clone(&self.transport);
        let target_speed_uri = self.target_speed_uri.clone();
        
        let listener = TargetSpeedListener::new(desired_velocity);
        transport.register_listener(&target_speed_uri, None, Arc::new(listener)).await?;
        
        info!("Target Speed subscriber registered");
        Ok(())
    }
    
    async fn setup_engage_subscriber(&self) -> Result<(), Box<dyn std::error::Error>> {
        let is_engaged = Arc::clone(&self.is_engaged);
        let pid_active = Arc::clone(&self.pid_active);
        let controller = Arc::clone(&self.controller);
        let transport = Arc::clone(&self.transport);
        let engage_uri = self.engage_uri.clone();
        
        let listener = EngageListener::new(is_engaged, pid_active, controller);
        transport.register_listener(&engage_uri, None, Arc::new(listener)).await?;
        
        info!("Engage subscriber registered");
        Ok(())
    }

    async fn setup_lidar_subscriber(&self) -> Result<(), Box<dyn std::error::Error>> {
        let latest_lidar_data = Arc::clone(&self.latest_lidar_data);
        let transport = Arc::clone(&self.transport);
        let lidar_uri = self.lidar_uri.clone();
        
        let listener = LidarListener::new(latest_lidar_data);
        transport.register_listener(&lidar_uri, None, Arc::new(listener)).await?;
        
        info!("Lidar subscriber registered for URI: {}", lidar_uri.to_uri(false));
        Ok(())
    }

    async fn setup_control_values_subscriber(&self) -> Result<(), Box<dyn std::error::Error>> {
        let throttle = Arc::clone(&self.throttle);
        let steer = Arc::clone(&self.steer);
        let brake = Arc::clone(&self.brake);
        let transport = Arc::clone(&self.transport);
        let control_values_uri = self.control_values_uri.clone();
        let listener = ControlValuesListener::new(throttle, steer, brake);
        transport.register_listener(&control_values_uri, None, Arc::new(listener)).await?;
        info!("Control Values subscriber registered for URI: {}", control_values_uri.to_uri(false));
        Ok(())
    }

    // Static method for PID computation and publishing
    async fn publish_acc(
        desired_velocity: &Arc<Mutex<f64>>,
        current_velocity: &Arc<Mutex<f64>>,
        current_time: &Arc<Mutex<f64>>,
        previous_time: &Arc<Mutex<f64>>,
        pid_active: &Arc<Mutex<bool>>,
        controller: &Arc<Mutex<PIDController>>,
        transport: &Arc<UPTransportZenoh>,
        actuation_uri: UUri,
        results: &Arc<Mutex<HashMap<String, Vec<f64>>>>,
        latest_lidar_data: &Arc<Mutex<Option<LidarMeasurement>>>,
        is_engaged: &Arc<Mutex<u8>>,
        engage_uri: &UUri,
        throttle: &Arc<Mutex<f64>>,
        steer: &Arc<Mutex<f64>>,
        brake: &Arc<Mutex<f64>>,
    ) {
        // Check if PID is active
        let is_active = {
            let active = pid_active.lock().unwrap();
            *active
        };
        
        if !is_active {
            return;
        }

        let (desired_vel, current_vel, curr_time) = {
            let desired = desired_velocity.lock().unwrap();
            let current = current_velocity.lock().unwrap();
            let time = current_time.lock().unwrap();
            (*desired, *current, *time)
        };

        // Compute acceleration using PID controller
        let (acceleration, emergency_brake_engaged, manual_brake_detected, cruise_should_disengage, cruise_can_reengage) = {
            let mut pid = controller.lock().unwrap();
            let lidar_data = latest_lidar_data.lock().unwrap();
            
            // Get current control values
            let throttle_input = *throttle.lock().unwrap();
            let steer_input = *steer.lock().unwrap();
            let brake_input = *brake.lock().unwrap();
            
            // Pass lidar data and control values to PID controller
            let lidar_ref = lidar_data.as_ref();
            
            match pid.compute(desired_vel, current_vel, curr_time, lidar_ref, throttle_input, steer_input, brake_input) {
                Ok(result) => {
                    if result.emergency_brake_engaged {
                        warn!("EMERGENCY BRAKE ENGAGED: {}", 
                              result.emergency_reason.as_ref().unwrap_or(&"Unknown reason".to_string()));
                    }
                    if result.manual_brake_detected {
                        info!("MANUAL BRAKE DETECTED: Driver intervention detected");
                    }
                    (result.acceleration, result.emergency_brake_engaged, result.manual_brake_detected, 
                     result.cruise_should_disengage, result.cruise_can_reengage)
                },
                Err(e) => {
                    error!("PID computation failed: {}", e);
                    return;
                }
            }
        };
        
        // Handle cruise control disengagement and re-engagement
        if cruise_should_disengage {
            let reason = if emergency_brake_engaged {
                "Emergency brake triggered"
            } else if manual_brake_detected {
                "Manual brake detected"
            } else {
                "Safety intervention"
            };
            
            info!("CRUISE CONTROL DISENGAGEMENT: {} - disengaging cruise control for safety", reason);
            {
                let mut engaged_state = is_engaged.lock().unwrap();
                *engaged_state = 0; // Disengage cruise control
            }
            {
                let mut active_state = pid_active.lock().unwrap();
                *active_state = false; // Deactivate PID control
            }
            
            // Publish disengage message to cruise control system
            let disengage_payload = "0";
            let disengage_message = UMessageBuilder::publish(engage_uri.clone())
                .build_with_payload(disengage_payload.to_string(), UPayloadFormat::UPAYLOAD_FORMAT_TEXT)
                .expect("Failed to build disengage message");
            
            if let Err(e) = transport.send(disengage_message).await {
                error!("Failed to send cruise control disengage message: {}", e);
            } else {
                info!("Successfully sent cruise control disengage message due to {}", reason);
            }
        }
        
        // Handle cruise control re-engagement
        if cruise_can_reengage {
            let current_engaged = {
                let engaged_state = is_engaged.lock().unwrap();
                *engaged_state
            };
            
            if current_engaged == 0 {
                info!("CRUISE CONTROL RE-ENGAGEMENT: Conditions met - re-engaging cruise control");
                {
                    let mut engaged_state = is_engaged.lock().unwrap();
                    *engaged_state = 1; // Re-engage cruise control
                }
                {
                    let mut active_state = pid_active.lock().unwrap();
                    *active_state = true; // Reactivate PID control
                }
                
                // Publish re-engage message to cruise control system
                let engage_payload = "1";
                let engage_message = UMessageBuilder::publish(engage_uri.clone())
                    .build_with_payload(engage_payload.to_string(), UPayloadFormat::UPAYLOAD_FORMAT_TEXT)
                    .expect("Failed to build engage message");
                
                if let Err(e) = transport.send(engage_message).await {
                    error!("Failed to send cruise control re-engage message: {}", e);
                } else {
                    info!("Successfully sent cruise control re-engage message");
                }
            }
        }
        
        if desired_vel < current_vel {
            debug!("Deceleration required");
        }

        // Create and publish uProtocol message
        let actuation_cmd_payload = format!("{}", acceleration);
        let message = UMessageBuilder::publish(actuation_uri)
            .build_with_payload(actuation_cmd_payload.clone(), UPayloadFormat::UPAYLOAD_FORMAT_TEXT)
            .unwrap();
        
        if let Err(e) = transport.send(message).await {
            error!("Failed to publish acceleration: {}", e);
        } else {
            debug!("Publishing Acceleration: {}", actuation_cmd_payload);
        }

        // Store results for later analysis
        {
            let mut results_guard = results.lock().unwrap();
            results_guard.get_mut("desired_velocity").unwrap().push(desired_vel);
            results_guard.get_mut("current_velocity").unwrap().push(current_vel);
            results_guard.get_mut("current_time").unwrap().push(curr_time);
            results_guard.get_mut("acceleration").unwrap().push(acceleration);
        }

        // Calculate and log delta time
        let (_prev_time, delta_time) = {
            let mut prev = previous_time.lock().unwrap();
            let delta = if *prev > 0.0 { curr_time - *prev } else { 0.0 };
            *prev = curr_time;
            (*prev, delta)
        };
        
        if delta_time > 0.0 {
            debug!("Delta time: {} seconds", delta_time);
        }
    }

    // Activation method
    fn activate_pid(
        pid_active: &Arc<Mutex<bool>>,
        controller: &Arc<Mutex<PIDController>>,
    ) {
        {
            let mut active = pid_active.lock().unwrap();
            *active = true;
        }
        {
            let mut pid = controller.lock().unwrap();
            pid.reset();
        }
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        info!("[INFO] PID controller ACTIVATED at {}", timestamp);
    }

    // Deactivation method
    fn deactivate_pid(
        pid_active: &Arc<Mutex<bool>>,
        controller: &Arc<Mutex<PIDController>>,
    ) {
        {
            let mut active = pid_active.lock().unwrap();
            *active = false;
        }
        {
            let mut pid = controller.lock().unwrap();
            pid.reset();
        }
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        info!("[INFO] PID controller DEACTIVATED at {}", timestamp);
    }
    
    pub fn store_results(&self) {
        let results = self.results.lock().unwrap();
        
        // Create logs directory if it doesn't exist
        if let Err(e) = std::fs::create_dir_all("logs") {
            error!("Failed to create logs directory: {}", e);
            return;
        }
        
        // Store each result type in separate files
        for (key, values) in results.iter() {
            let filename = format!("logs/{}.log", key);
            let content = values.iter()
                .map(|v| v.to_string())
                .collect::<Vec<String>>()
                .join("\n");
            
            if let Err(e) = std::fs::write(&filename, content) {
                error!("Failed to write {}: {}", filename, e);
            } else {
                info!("Results saved to {}", filename);
            }
        }

        // Also save as JSON for compatibility
        if let Ok(json) = serde_json::to_string(&*results) {
            std::fs::write("logs/pid_results.json", json).unwrap_or_else(|e| {
                error!("Failed to write JSON results: {}", e);
            });
        }
    }
    
    pub fn show_results(&self) {
        let results = self.results.lock().unwrap();
        
        info!("PID Controller Results Summary:");
        
        if let (Some(desired), Some(current), Some(acceleration)) = (
            results.get("desired_velocity"),
            results.get("current_velocity"), 
            results.get("acceleration")
        ) {
            let data_points = desired.len().min(current.len()).min(acceleration.len());
            info!("Total data points: {}", data_points);
            
            if data_points > 0 {
                let mut min_error = f64::MAX;
                let mut max_error = f64::MIN;
                let mut sum_error = 0.0;
                
                for i in 0..data_points {
                    let error = desired[i] - current[i];
                    min_error = min_error.min(error);
                    max_error = max_error.max(error);
                    sum_error += error;
                }
                
                let avg_error = sum_error / data_points as f64;
                
                info!("Min error: {:.4}", min_error);
                info!("Max error: {:.4}", max_error);
                info!("Avg error: {:.4}", avg_error);
                
                if let Some(acc_values) = results.get("acceleration") {
                    if !acc_values.is_empty() {
                        let min_acc = acc_values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
                        let max_acc = acc_values.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
                        let avg_acc = acc_values.iter().sum::<f64>() / acc_values.len() as f64;
                        
                        info!("Acceleration - Min: {:.4}, Max: {:.4}, Avg: {:.4}", min_acc, max_acc, avg_acc);
                    }
                }
            }
        } else {
            info!("No data points available");
        }
    }

    // Additional helper method to get current PID status
    #[allow(dead_code)]    
    pub fn is_active(&self) -> bool {
        let active = self.pid_active.lock().unwrap();
        *active
    }

    // Get current state for debugging
    #[allow(dead_code)]    
    pub fn get_state(&self) -> (f64, f64, f64, bool) {
        let current_vel = *self.current_velocity.lock().unwrap();
        let desired_vel = *self.desired_velocity.lock().unwrap();
        let current_time = *self.current_time.lock().unwrap();
        let is_active = *self.pid_active.lock().unwrap();
        
        (current_vel, desired_vel, current_time, is_active)
    }

    // Get current control values (throttle, steer, brake)
    pub fn get_control_values(&self) -> (f64, f64, f64) {
        let throttle = *self.throttle.lock().unwrap();
        let steer = *self.steer.lock().unwrap();
        let brake = *self.brake.lock().unwrap();
        (throttle, steer, brake)
    }
}

// Listener implementations
struct ClockListener {
    current_time: Arc<Mutex<f64>>,
}

impl ClockListener {
    fn new(current_time: Arc<Mutex<f64>>) -> Self {
        Self { current_time }
    }
}

#[async_trait::async_trait]
impl UListener for ClockListener {
    async fn on_receive(&self, message: UMessage) {
        if let Some(payload) = message.payload {
            let bytes = &payload[..];
            
            // Try to parse as text first (new format)
            let time_value = if let Ok(payload_str) = std::str::from_utf8(&bytes) {
                match payload_str.trim().parse::<f64>() {
                    Ok(time) => time,
                    Err(_) => {
                        // Fall back to JSON format for backward compatibility
                        if let Ok(clock_status) = serde_json::from_slice::<ClockStatus>(&bytes) {
                            clock_status.time
                        } else {
                            error!("[ERROR] Timestamp processing failed as JSON");
                            return;
                        }
                    }
                }
            } else {
                error!("[ERROR] Timestamp processing failed as UTF-8");
                return;
            };
            
            {
                let mut clock = self.current_time.lock().unwrap();
                *clock = time_value;
            }
            debug!("Received current clock '{:.4}' seconds", time_value);
        }
    }
}

struct VelocityListener {
    current_velocity: Arc<Mutex<f64>>,
    desired_velocity: Arc<Mutex<f64>>,
    current_time: Arc<Mutex<f64>>,
    previous_time: Arc<Mutex<f64>>,
    pid_active: Arc<Mutex<bool>>,
    controller: Arc<Mutex<PIDController>>,
    results: Arc<Mutex<HashMap<String, Vec<f64>>>>,
    actuation_uri: UUri,
    transport: Arc<UPTransportZenoh>,
    latest_lidar_data: Arc<Mutex<Option<LidarMeasurement>>>,
    is_engaged: Arc<Mutex<u8>>,
    engage_uri: UUri,
    throttle: Arc<Mutex<f64>>,
    steer: Arc<Mutex<f64>>,
    brake: Arc<Mutex<f64>>,
}

impl VelocityListener {
    fn new(
        current_velocity: Arc<Mutex<f64>>,
        desired_velocity: Arc<Mutex<f64>>,
        current_time: Arc<Mutex<f64>>,
        previous_time: Arc<Mutex<f64>>,
        pid_active: Arc<Mutex<bool>>,
        controller: Arc<Mutex<PIDController>>,
        results: Arc<Mutex<HashMap<String, Vec<f64>>>>,
        actuation_uri: UUri,
        transport: Arc<UPTransportZenoh>,
        latest_lidar_data: Arc<Mutex<Option<LidarMeasurement>>>,
        is_engaged: Arc<Mutex<u8>>,
        engage_uri: UUri,
        throttle: Arc<Mutex<f64>>,
        steer: Arc<Mutex<f64>>,
        brake: Arc<Mutex<f64>>,
    ) -> Self {
        Self {
            current_velocity,
            desired_velocity,
            current_time,
            previous_time,
            pid_active,
            controller,
            results,
            actuation_uri,
            transport,
            latest_lidar_data,
            is_engaged,
            engage_uri,
            throttle,
            steer,
            brake,
        }
    }
}

#[async_trait::async_trait]
impl UListener for VelocityListener {
    async fn on_receive(&self, message: UMessage) {
        if let Some(payload) = message.payload {
            let bytes = &payload[..];
            
            // Try to parse as text first (new format)
            let velocity_value = if let Ok(payload_str) = std::str::from_utf8(&bytes) {
                match payload_str.trim().parse::<f64>() {
                    Ok(velocity) => velocity,
                    Err(_) => {
                        // Fall back to JSON format for backward compatibility
                        if let Ok(velocity_status) = serde_json::from_slice::<VelocityStatus>(&bytes) {
                            velocity_status.velocity
                        } else {
                            error!("Failed to parse velocity payload");
                            return;
                        }
                    }
                }
            } else {
                error!("Failed to parse velocity payload as UTF-8");
                return;
            };
            
            {
                let mut vel = self.current_velocity.lock().unwrap();
                *vel = velocity_value;
            }
            debug!("Received current velocity '{:.2}'", velocity_value);
            
            // Trigger PID computation
            UProtocolHandler::publish_acc(
                &self.desired_velocity,
                &self.current_velocity,
                &self.current_time,
                &self.previous_time,
                &self.pid_active,
                &self.controller,
                &self.transport,
                self.actuation_uri.clone(),
                &self.results,
                &self.latest_lidar_data,
                &self.is_engaged,
                &self.engage_uri,
                &self.throttle,
                &self.steer,
                &self.brake,
            ).await;
        }
    }
}

struct TargetSpeedListener {
    desired_velocity: Arc<Mutex<f64>>,
}

impl TargetSpeedListener {
    fn new(desired_velocity: Arc<Mutex<f64>>) -> Self {
        Self { desired_velocity }
    }
}

#[async_trait::async_trait]
impl UListener for TargetSpeedListener {
    async fn on_receive(&self, message: UMessage) {
        if let Some(payload) = message.payload {
            let bytes = &payload[..];
            
            let speed_value = if let Ok(target_speed) = serde_json::from_slice::<TargetSpeed>(&bytes) {
                target_speed.speed
            } else if let Ok(payload_str) = std::str::from_utf8(&bytes) {
                match payload_str.trim().parse::<f64>() {
                    Ok(speed) => speed,
                    Err(_) => {
                        error!("Failed to parse target speed: {}", payload_str);
                        return;
                    }
                }
            } else {
                error!("Failed to parse target speed payload");
                return;
            };
            
            {
                let mut vel = self.desired_velocity.lock().unwrap();
                *vel = speed_value;
            }
            info!("Received desired velocity '{:.2}'", speed_value);
        }
    }
}

struct EngageListener {
    is_engaged: Arc<Mutex<u8>>,
    pid_active: Arc<Mutex<bool>>,
    controller: Arc<Mutex<PIDController>>,
}

impl EngageListener {
    fn new(
        is_engaged: Arc<Mutex<u8>>,
        pid_active: Arc<Mutex<bool>>,
        controller: Arc<Mutex<PIDController>>,
    ) -> Self {
        Self {
            is_engaged,
            pid_active,
            controller,
        }
    }
}

#[async_trait::async_trait]
impl UListener for EngageListener {
    async fn on_receive(&self, message: UMessage) {
        if let Some(payload) = message.payload {
            let bytes = &payload[..];
            
            // Try to parse as text first (new format)
            let engaged_value = if let Ok(payload_str) = std::str::from_utf8(&bytes) {
                match payload_str.trim().parse::<u8>() {
                    Ok(engaged) => engaged,
                    Err(_) => {
                        // Fall back to JSON format for backward compatibility
                        if let Ok(engage_status) = serde_json::from_slice::<EngageStatus>(&bytes) {
                            engage_status.engaged
                        } else {
                            error!("Failed to parse engage status payload");
                            return;
                        }
                    }
                }
            } else {
                error!("Failed to parse engage status payload as UTF-8");
                return;
            };
            
            let _was_engaged;
            {
                let mut engaged_state = self.is_engaged.lock().unwrap();
                _was_engaged = *engaged_state;
                *engaged_state = engaged_value;
            }
            
            info!("Received engage status: {}", engaged_value);
            
            // Handle activation/deactivation
            let enable = engaged_value != 0;
            let was_active = {
                let active = self.pid_active.lock().unwrap();
                *active
            };
            
            if enable && !was_active {
                UProtocolHandler::activate_pid(&self.pid_active, &self.controller);
            } else if !enable && was_active {
                UProtocolHandler::deactivate_pid(&self.pid_active, &self.controller);
            }
        }
    }
}

// Lidar Listener struct
struct LidarListener {
    latest_lidar_data: Arc<Mutex<Option<LidarMeasurement>>>,
}

impl LidarListener {
    fn new(latest_lidar_data: Arc<Mutex<Option<LidarMeasurement>>>) -> Self {
        Self {
            latest_lidar_data,
        }
    }
}

#[async_trait::async_trait]
impl UListener for LidarListener {
    async fn on_receive(&self, message: UMessage) {
        if let Some(payload) = message.payload {
            let bytes = &payload[..];
            
            // First, let's see what the JSON actually looks like
            if let Ok(json_str) = std::str::from_utf8(&bytes) {
                debug!("Raw lidar JSON: {}", json_str.chars().take(500).collect::<String>());
                
                // Try to parse as our expected structure first
                match serde_json::from_slice::<LidarMeasurement>(&bytes) {
                    Ok(lidar_measurement) => {
                        let detection_count = lidar_measurement.detections.len();                        
                        // Store the latest lidar data
                        {
                            let mut lidar_data = self.latest_lidar_data.lock().unwrap();
                            *lidar_data = Some(lidar_measurement);
                        }
                        
                        // Optional: Print some sample detections for debugging
                        debug!("First few lidar detections (if any):");
                        if let Ok(lidar_data) = serde_json::from_slice::<LidarMeasurement>(&bytes) {
                            for (i, detection) in lidar_data.detections.iter().take(3).enumerate() {
                                debug!("  Detection {}: x={:.2}, y={:.2}, z={:.2}, intensity={:.3}", 
                                       i, detection.point.x, detection.point.y, detection.point.z, detection.intensity);
                            }
                        }
                    }
                    Err(e) => {
                        // Try to parse as a generic JSON value to understand the structure
                        match serde_json::from_slice::<serde_json::Value>(&bytes) {
                            Ok(json_value) => {
                                error!("Failed to parse as LidarMeasurement: {}. Structure: {:?}", 
                                       e, json_value.as_object().map(|obj| obj.keys().collect::<Vec<_>>()));
                                debug!("Sample JSON structure: {}", serde_json::to_string_pretty(&json_value).unwrap_or_else(|_| "Could not pretty print".to_string()).chars().take(1000).collect::<String>());
                            }
                            Err(_) => {
                                error!("Failed to parse lidar measurement: {}", e);
                            }
                        }
                    }
                }
            } else {
                error!("Lidar payload is not valid UTF-8");
            }
        }
    }
}

struct ControlValuesListener {
    throttle: Arc<Mutex<f64>>,
    steer: Arc<Mutex<f64>>,
    brake: Arc<Mutex<f64>>,
}

impl ControlValuesListener {
    fn new(throttle: Arc<Mutex<f64>>, steer: Arc<Mutex<f64>>, brake: Arc<Mutex<f64>>) -> Self {
        Self { throttle, steer, brake }
    }
}

#[async_trait::async_trait]
impl UListener for ControlValuesListener {
    async fn on_receive(&self, message: UMessage) {
        if let Some(payload) = message.payload {
            let bytes = &payload[..];
            match serde_json::from_slice::<ControlValues>(bytes) {
                Ok(control) => {
                    *self.throttle.lock().unwrap() = control.throttle;
                    *self.steer.lock().unwrap() = control.steer;
                    *self.brake.lock().unwrap() = control.brake;
                    info!("Received control values: throttle={:.3}, steer={:.3}, brake={:.3}", control.throttle, control.steer, control.brake);
                },
                Err(e) => {
                    error!("Failed to parse control values JSON: {}", e);
                }
            }
        }
    }
}
