

from pymavlink import mavutil
import time
import sys
import json
import argparse
from datetime import datetime
import requests




INTERCEPT = -24.10
COEFFICIENT = 0.004130


# MONITOR PARAMETERS
WEIGHT_THRESHOLD_KG = 10.0
CLIMB_ALTITUDE_START = 3.0      # meters
CLIMB_ALTITUDE_END = 35.0       # meters
MIN_CLIMB_RATE = 0.5            # m/s
DATA_COLLECTION_SECONDS = 2.5
SAMPLE_INTERVAL_MS = 100


#+++++++++++++++++++++++++++++++++++++++++
# Battery channel, Possibly set to two.
BATTERY_INSTANCE = 0            # 0=battery1, 1=battery2, 2=total
#++++++++++++++++++++++++++++++


# REALITY CHECK BOUNDS
MIN_POWER_W = 50
MAX_POWER_W = 50000
MIN_WEIGHT_KG = -2.0
MAX_WEIGHT_KG = 50.0


# GALE Cloud Service Configuration
GALE_CLOUD_ENDPOINT = "https://your-gale-cloud-service.com/api/weight"
GALE_CLOUD_ENABLED = False  # Set to True to enable cloud uploads
GALE_CLOUD_TIMEOUT = 5


# Local logging
LOG_FILE = "/var/log/weight_monitor.log"
LOG_ENABLED = False


# ============================================================================
# STATE CLASS
# ============================================================================
class WeightMonitorState:
    def __init__(self):
        self.monitoring = False
        self.collecting = False
        self.alerted = False
        self.power_samples = []
        self.required_samples = int(DATA_COLLECTION_SECONDS * 1000 / SAMPLE_INTERVAL_MS)
       
        # Current vehicle state
        self.armed = False
        self.altitude = 0.0
        self.climb_rate = 0.0
        self.voltage = 0.0
        self.current = 0.0
        self.voltage1 = 0.0
        self.current1 = 0.0
        self.voltage2 = 0.0
        self.current2 = 0.0
       
        # Timestamps for rate limiting
        self.last_sample_time = 0
        self.flight_start_time = None
        self.estimation_count = 0




def send_statustext_to_gcs(master, message, severity=mavutil.mavlink.MAV_SEVERITY_WARNING):
   
    # STATUSTEXT messages are max 50 characters
    message = message[:50]
   
    master.mav.statustext_send(
        severity,
        message.encode('utf-8')
    )
    log_to_file(f"Sent to GCS: {message}")


def send_weight_parameter(master, weight):
 
   
    master.mav.named_value_float_send(
        0,  # time_boot_ms (0 = use system time)
        b'WEIGHT_KG',              # parameter name (max 10 chars)
        weight                     # value in kg
    )


#LOG NOT SURE IF THIS IS WORKING
def log_to_file(message, level="INFO"):
    """Write log message to file"""
    if not LOG_ENABLED:
        return
   
    try:
        timestamp = datetime.now().isoformat()
        with open(LOG_FILE, 'a') as f:
            f.write(f"{timestamp} [{level}] {message}\n")
    except Exception as e:
        print(f"Logging error: {e}")


def log_weight_estimate(data):
    """Log weight estimation data in JSON format"""
    if not LOG_ENABLED:
        return
   
    try:
        log_file = LOG_FILE.replace('.log', '_estimates.json')
        with open(log_file, 'a') as f:
            json.dump(data, f)
            f.write('\n')
    except Exception as e:
        print(f"Data logging error: {e}")


# ============================================================================
# GALE CLOUD INTEGRATION # AGAIN NOT SURE IF WOKRIGN
# ============================================================================
def send_to_gale_cloud(data):
   
    if not GALE_CLOUD_ENABLED:
        return False
   
    try:
        response = requests.post(
            GALE_CLOUD_ENDPOINT,
            json=data,
            timeout=GALE_CLOUD_TIMEOUT,
            headers={'Content-Type': 'application/json'}
        )
       
        if response.status_code == 200:
            print(f" Data sent to GALE Cloud (status: {response.status_code})")
            log_to_file(f"Cloud upload successful: {response.status_code}", "INFO")
            return True
        else:
            print(f" Cloud upload failed (status: {response.status_code})")
            log_to_file(f"Cloud upload failed: {response.status_code}", "WARNING")
            return False
           
    except requests.Timeout:
        print(" Cloud upload timeout")
        log_to_file("Cloud upload timeout", "WARNING")
        return False
    except Exception as e:
        print(f" Cloud upload error: {e}")
        log_to_file(f"Cloud upload error: {e}", "ERROR")
        return False


# ============================================================================
# MAVLINK CONNECTION
# ============================================================================
# CONNECTION USING UDP
def connect_mavlink(connection_string):
    """Connect to flight controller via UDP MAVLink (through MAV Router)"""
    print(f"Connecting to MAVRouter on {connection_string}")
    log_to_file(f"Connecting to {connection_string}")
   
    master = mavutil.mavlink_connection(connection_string)
   
    print("Waiting for heartbeat")
    master.wait_heartbeat()
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    log_to_file(f"Connected to system {master.target_system}, component {master.target_component}")
   
    return master


def request_message_interval(master, message_id, frequency_hz):
    """REQUESTING MAVLINK DATA AT A CERTAIN FREQUENCY"""
    interval_us = int(1000000 / frequency_hz) if frequency_hz > 0 else -1
   
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,  # Message ID
        interval_us,  # Interval in microseconds
        0, 0, 0, 0, 0
    )


def setup_streams(master):
   
   
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)      # HEARTBEAT at 1 Hz
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 10)     # SYS_STATUS at 10 Hz
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 10) # battery status at 10 hz
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 10)       # VFR_HUD at 10 HZ to get altitude and climb data
    request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10) # GPS to get alt
   
    print("Stream configuration complete")
    log_to_file("Stream configuration complete")
    time.sleep(1)  # Give time for streams to start


# ============================================================================
# MESSAGE HANDLERS
# ============================================================================
def handle_heartbeat(msg, state):
    """Handle HEARTBEAT message"""
    was_armed = state.armed
    # Check armed state
    state.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
   
    # note for me, using this, base mode stores bit locations and values to flag desirable things we need.
    # 128 decimal so 10000000 bitmask and operation determines if drone is armed
    # this is similar to CRC have previously written in C#
    # converts to boolean
   
    if state.armed and not was_armed:
        state.flight_start_time = time.time()
        log_to_file("Vehicle ARMED - flight started")


def handle_sys_status(msg, state):
    """Handle SYS_STATUS message"""
    state.voltage = msg.voltage_battery / 1000.0  # mV to V
    state.current = msg.current_battery / 100.0   # centiamps to amps


def handle_battery_status(msg, state):
    """Handle BATTERY_STATUS message"""
    if msg.id == 0:  # Battery 1
        if len(msg.voltages) > 0 and msg.voltages[0] != 65535:
            state.voltage1 = sum([v/1000.0 for v in msg.voltages if v != 65535])
        state.current1 = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
    elif msg.id == 1:  # Battery 2
        if len(msg.voltages) > 0 and msg.voltages[0] != 65535:
            state.voltage2 = sum([v/1000.0 for v in msg.voltages if v != 65535])
        state.current2 = msg.current_battery / 100.0 if msg.current_battery != -1 else 0


def handle_vfr_hud(msg, state):
    """Handle VFR_HUD message"""
    state.climb_rate = msg.climb


def handle_global_position_int(msg, state):
    """Handle GLOBAL_POSITION_INT message"""
    state.altitude = msg.relative_alt / 1000.0  # mm to m


#WEIGHT
def get_battery_power(state):
    """Get current power draw based on configured battery instance"""
    if BATTERY_INSTANCE == 0:
        voltage = state.voltage1
        current = state.current1
    elif BATTERY_INSTANCE == 1:
        voltage = state.voltage2
        current = state.current2
    else:  # Total
        voltage = state.voltage
        current = state.current
   
    if voltage > 0 and current > 0:
        return voltage * current
    return None


def process_weight_monitoring(state, master):
    """Main weight monitoring logic with cloud integration and Mission Planner alerts"""
    current_time = time.time()
   
    # Reset if disarmed
    if not state.armed:
        if state.monitoring:
            print("DISARMED  resetting state")
            log_to_file("Vehicle disarmed  resetting monitoring state")
        state.monitoring = False
        state.collecting = False
        state.alerted = False
        state.power_samples = []
        return
   
    # Check if in climb band
    in_climb_band = (
        CLIMB_ALTITUDE_START <= state.altitude <= CLIMB_ALTITUDE_END and
        state.climb_rate >= MIN_CLIMB_RATE
    )
   
    # Start monitoring
    if in_climb_band and not state.monitoring:
        print("=" * 70)
        print("CLIMB DETECTED - starting weight estimation")
        print(f"Altitude: {state.altitude:.1f}m, Climb rate: {state.climb_rate:.1f}m/s")
        print("=" * 70)
        log_to_file(f"Climb detected at {state.altitude:.1f}m, rate {state.climb_rate:.1f}m/s")
        state.monitoring = True
        state.collecting = True
        state.power_samples = []
        state.last_sample_time = current_time
   
    # Sample collection
    if state.collecting:
        # Rate limit sampling
        if (current_time - state.last_sample_time) * 1000 >= SAMPLE_INTERVAL_MS:
            power = get_battery_power(state)
           
            if power is not None and MIN_POWER_W <= power <= MAX_POWER_W:
                state.power_samples.append(power)
           
            state.last_sample_time = current_time
       
        # Stop conditions
        if len(state.power_samples) >= state.required_samples or not in_climb_band:
            state.collecting = False
           
            if len(state.power_samples) == 0:
                print("ERROR: No valid power samples collected")
                log_to_file("No valid power samples collected", "ERROR")
                state.monitoring = False
                return
           
            # Calculate estimated weight
            avg_power = sum(state.power_samples) / len(state.power_samples)
            estimated_weight = INTERCEPT + COEFFICIENT * avg_power
            state.estimation_count += 1
           
            print("\n" + "=" * 70)
            print("WEIGHT ESTIMATION COMPLETE")
            print(f"Samples collected: {len(state.power_samples)}")
            print(f"Average Power: {avg_power:.1f} Watts")
            print(f"Estimated Weight: {estimated_weight:.2f} kg")
            print("=" * 70)
           
            # Send weight to Mission Planner HUD
            send_weight_parameter(master, estimated_weight)
           
            # Send info message to Mission Planner
            send_statustext_to_gcs(master,
                                   f"Weight: {estimated_weight:.2f}kg from {avg_power:.0f}W",
                                   mavutil.mavlink.MAV_SEVERITY_INFO)
           
            # Prepare data package
            estimation_data = {
                'timestamp': datetime.now().isoformat(),
                'flight_time_seconds': time.time() - state.flight_start_time if state.flight_start_time else 0,
                'estimation_number': state.estimation_count,
                'samples_collected': len(state.power_samples),
                'average_power_w': round(avg_power, 2),
                'estimated_weight_kg': round(estimated_weight, 2),
                'threshold_kg': WEIGHT_THRESHOLD_KG,
                'altitude_m': round(state.altitude, 2),
                'climb_rate_ms': round(state.climb_rate, 2),
                'overweight_alert': estimated_weight > WEIGHT_THRESHOLD_KG,
                'within_bounds': MIN_WEIGHT_KG <= estimated_weight <= MAX_WEIGHT_KG,
                'battery_instance': BATTERY_INSTANCE
            }
           
            # Log to file
            log_weight_estimate(estimation_data)
            log_to_file(f"Weight estimated: {estimated_weight:.2f}kg from {avg_power:.1f}W")
           
            # Send to GALE Cloud
            if GALE_CLOUD_ENABLED:
                send_to_gale_cloud(estimation_data)
           
            # REALITY check
            if not (MIN_WEIGHT_KG <= estimated_weight <= MAX_WEIGHT_KG):
                print(f"WARNING: Estimated weight outside physical bounds "
                      f"({MIN_WEIGHT_KG}-{MAX_WEIGHT_KG} kg)")
                log_to_file(f"Weight estimate out of bounds: {estimated_weight:.2f}kg", "WARNING")
                # Send error to Mission Planner
                send_statustext_to_gcs(master,
                                       f"Weight calc error: {estimated_weight:.2f}kg invalid",
                                       mavutil.mavlink.MAV_SEVERITY_ERROR)
           
            # Alert if overweight
            # ... inside process_weight_monitoring ...


            # Alert if overweight
            elif estimated_weight > WEIGHT_THRESHOLD_KG and not state.alerted:
                print("\n" + "!" * 70)
                print("!!! OVERWEIGHT WARNING !!!")
                print(f"Estimate: {estimated_weight:.2f} kg   Limit: {WEIGHT_THRESHOLD_KG:.1f} kg")
                print("!" * 70 + "\n")
                log_to_file(f"OVERWEIGHT ALERT: {estimated_weight:.2f}kg > {WEIGHT_THRESHOLD_KG}kg", "ALERT")
               
                # --- CHANGE START: FORCE POPUP/AUDIO ALERT ---
                # Send 3 times to ensure Mission Planner speaks it and flashes it clearly
                alert_msg = f"ALERT: OVERWEIGHT DETECTED! {estimated_weight:.1f}kg"
               
                for _ in range(3):
                    send_statustext_to_gcs(master,
                                           alert_msg,
                                           mavutil.mavlink.MAV_SEVERITY_ALERT) # Severity 1 = Red Flash + Audio
                    time.sleep(0.2) # Small delay between repeats
                # --- CHANGE END ---


                state.alerted = True
            else:
                print("✓ Weight within limits\n")
                log_to_file(f"Weight within limits: {estimated_weight:.2f}kg", "INFO")
           
            state.monitoring = False


# ============================================================================
# MAIN
# ============================================================================
def main():
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    # CHANGE UDP PORT AS NECESSARY
    parser = argparse.ArgumentParser(description='MAVLink Weight Monitor with GALE Integration - UDP via MAV Router')
    parser.add_argument('--connect', type=str, default='udp:127.0.0.1:14551',
                       help='UDP connection to MAV Router (default: udp:127.0.0.1:14551)')
    parser.add_argument('--cloud', action='store_true',
                       help='Enable GALE Cloud uploads')
    parser.add_argument('--cloud-endpoint', type=str,
                       help='Override GALE Cloud endpoint URL')
    args = parser.parse_args()
   
    # Override cloud settings if provided
    global GALE_CLOUD_ENABLED, GALE_CLOUD_ENDPOINT
    if args.cloud:
        GALE_CLOUD_ENABLED = True
    if args.cloud_endpoint:
        GALE_CLOUD_ENDPOINT = args.cloud_endpoint
   
    print("=" * 70)
    print("WEIGHT MONITOR - GALE INTEGRATION")
    print(f"Weight = {INTERCEPT:.2f} + {COEFFICIENT:.6f} * Power")
    print(f"Threshold: {WEIGHT_THRESHOLD_KG:.1f} kg")
    print(f"Battery Instance: {BATTERY_INSTANCE} (0=bat1, 1=bat2, 2=total)")
    print(f"Cloud Uploads: {'ENABLED' if GALE_CLOUD_ENABLED else 'DISABLED'}")
    if GALE_CLOUD_ENABLED:
        print(f"Cloud Endpoint: {GALE_CLOUD_ENDPOINT}")
    print(f"Logging: {'ENABLED' if LOG_ENABLED else 'DISABLED'}")
    if LOG_ENABLED:
        print(f"Log File: {LOG_FILE}")
    print("=" * 70)
   
    log_to_file("=" * 50)
    log_to_file("Weight Monitor Started")
    log_to_file(f"Cloud uploads: {GALE_CLOUD_ENABLED}")
    log_to_file(f"Battery instance: {BATTERY_INSTANCE}")
   
    try:
        master = connect_mavlink(args.connect)
        setup_streams(master)
    except Exception as e:
        print(f"Failed to connect: {e}")
        log_to_file(f"Connection failed: {e}", "ERROR")
        sys.exit(1)
   
    state = WeightMonitorState()
   
    print("\nMonitoring started. Waiting for vehicle to arm and climb...\n")
    log_to_file("Monitoring loop started")
   
    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=0.01)
           
            if msg is not None:
                msg_type = msg.get_type()
               
                if msg_type == 'HEARTBEAT':
                    handle_heartbeat(msg, state)
                elif msg_type == 'SYS_STATUS':
                    handle_sys_status(msg, state)
                elif msg_type == 'BATTERY_STATUS':
                    handle_battery_status(msg, state)
                elif msg_type == 'VFR_HUD':
                    handle_vfr_hud(msg, state)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    handle_global_position_int(msg, state)
           
            process_weight_monitoring(state, master)
            time.sleep(0.001)
   
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        log_to_file("Weight Monitor shutting down (user interrupt)")
    except Exception as e:
        print(f"\nERROR: {e}")
        log_to_file(f"Fatal error: {e}", "ERROR")
        import traceback
        traceback.print_exc()
    finally:
        master.close()
        log_to_file("Connection closed")
        print("Connection closed")


if __name__ == "__main__":
    main()

