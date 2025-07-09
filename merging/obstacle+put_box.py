from machine import Pin, PWM
import time
from WaveShaker import *
from IMU_test import *
from ultrasonic import *
from encoders import Encoder

# Motor setup
freq = 1000
left_motor = PWM(Pin(14), freq=freq)
left_motor_back = PWM(Pin(12), freq=freq)
right_motor = PWM(Pin(27), freq=freq)
right_motor_back = PWM(Pin(26), freq=freq)

encoder_A = Encoder(18, 5)
encoder_B = Encoder(16, 17)

electromagnet = Pin(19, Pin.OUT)
electromagnet.off()

# --- PID Tuning Constants ---
KP_LINE         = 80.0
KI_LINE         = 0.5
KD_LINE         = 500.0
INTEGRAL_MAX    = 100
BASE_SPEED      = 700

robot_heading = 0

took_box = False

def set_speed(pwm, duty):
    pwm.duty(duty)

def forward(vel=500):
    apply_encoder_correction(vel, vel)
    
def backward(vel=500):
    apply_encoder_correction(-vel, -vel)
    
def left(vel=500):
    apply_encoder_correction(-vel, vel)

def right(vel=500):
    apply_encoder_correction(vel, -vel)
    
def Stop():
    set_speed(left_motor, 0)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, 0)

def forward_bit():
    #forward was 600
    forward(550)
    time.sleep(0.4)
    
def deep_copy_graph(graph):
    return {
        k: [tuple(edge) for edge in v]
        for k, v in graph.items()
    }

def line_follow_backward(duration=2.2):
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < int(duration * 1000):
        binary = ir.read_binary()

        weights = [-2, -1, 0, 1, 2]
        total_weighted_sum = 0
        black_sensors_count = 0
        for val, w in zip(binary, weights):
            if val == 0:
                total_weighted_sum += w
                black_sensors_count += 1
        err_line = (total_weighted_sum / black_sensors_count) if black_sensors_count else 0

        # Use same PID constants but reversed logic for backward
        proportional_term = KP_LINE * err_line
        integral_term = KI_LINE * err_line
        derivative_term = 0  # optional for short time

        adjust = proportional_term + integral_term + derivative_term

        speed_r = BASE_SPEED + adjust
        speed_l = BASE_SPEED - adjust

        # Use apply_encoder_correction with negative speeds
        apply_encoder_correction(-speed_l, -speed_r)

        time.sleep(0.02)
        #Stop()

def apply_encoder_correction(base_left, base_right):
    count_left = encoder_B.get_count()
    count_right = encoder_A.get_count()

    correction = (count_right - count_left) * 3  # Tune this gain

    speed_l = base_left - correction
    speed_r = base_right + correction

    # Set left wheel
    if speed_l >= 0:
        set_speed(left_motor, min(1023, int(speed_l)))
        set_speed(left_motor_back, 0)
    else:
        set_speed(left_motor, 0)
        set_speed(left_motor_back, min(1023, int(-speed_l)))

    # Set right wheel
    if speed_r >= 0:
        set_speed(right_motor, min(1023, int(speed_r)))
        set_speed(right_motor_back, 0)
    else:
        set_speed(right_motor, 0)
        set_speed(right_motor_back, min(1023, int(-speed_r)))

    encoder_A.reset()
    encoder_B.reset()

def turn_by_angle(deg, imu_tracker):
    start = imu_tracker.update_heading()
    print(f"Initial:{start}")
    target = (start + deg + 360) % 360
    
    print(f"target is:{target}")

    while True:
        speed = 0
        current = imu_tracker.update_heading()
        angle_diff = (target - current + 540) % 360 - 180
        #print(f"degree:{current}")
        #print(f"difference:{angle_diff}")
        
        if abs(angle_diff) <= 4:
            break

        if angle_diff < 0:
            if took_box:
                speed = 900
            else:
                speed = 600
                
            right(speed)
            time.sleep(0.1)
        else:
            if took_box:
                speed = 900
            else:
                speed = 600
            left(speed)
            time.sleep(0.1)
    #print(f"Turned {deg}° to heading {imu_tracker.update_heading():.2f}°")

def perform_turn(rel_turn, imu_tracker):
    global robot_heading
    if rel_turn == 0:
        print("Go forward_bit")
        forward_bit()
    elif rel_turn == 1:
        print("Turn right")
        forward_bit()
        turn_by_angle(-90, imu_tracker)
        imu_tracker.heading = 270
    elif rel_turn == 2:
        print("Turn back")
        turn_by_angle(180, imu_tracker)
        imu_tracker.heading = 180
    elif rel_turn == 3:
        print("Turn left")
        forward_bit()
        turn_by_angle(90, imu_tracker)
        imu_tracker.heading = 90
        
    robot_heading = (robot_heading + rel_turn) % 4

# --- Graph & Path Planning ---
graph = {
    'A1': [('S1', 1, 2)], 
    'A2': [('S2', 1, 2)],
    'A3': [('S3', 1, 2)],
    'A4': [('S4', 1, 2)],
    'S1': [('A1', 1, 0), ('S2', 1, 1), ('F', 5, 2)],
    'S2': [('A2', 1, 0), ('S3', 1, 1), ('S1', 1, 3)],
    'S3': [('A3', 1, 0), ('S4', 1, 1), ('S2', 1, 3)],
    'S4': [('A4', 1, 0), ('L', 2, 1), ('S3', 1, 3)],
    'L': [('K', 5, 1), ('M', 3, 2), ('S4', 2, 3)],
    'K': [('J', 3, 2), ('L', 5, 3)],
    'M': [('L', 3, 0), ('J', 5, 1), ('H', 2, 2)],
    'J': [('K', 3, 0), ('I', 2, 2), ('M', 5, 3)],
    'F': [('S1', 5, 0), ('H', 5, 1), ('E', 2, 2)],
    'H': [('M', 2, 0), ('I', 5, 1), ('O', 2, 2), ('F', 5, 3)],
    'I': [('J', 2, 0), ('G4', 5, 2), ('H', 5, 3)],
    'E': [('F', 2, 0), ('O', 5, 1), ('D', 3, 2)],
    'O': [('H', 2, 0), ('C', 3, 2), ('E', 5, 3)],
    'D': [('E', 3, 0), ('C', 5, 1)],
    'C': [('O', 3, 0), ('G1', 2, 1), ('D', 5, 3)],
    'G1': [('G2', 1, 1), ('B1', 1, 2), ('C', 2, 3)],
    'G2': [('G3', 1, 1), ('B2', 1, 2), ('G1', 1, 3)],
    'G3': [('G4', 1, 1), ('B3', 1, 2), ('G2', 1, 3)],
    'G4': [('I', 5, 0), ('B4', 1, 2), ('G3', 1, 3)],
    'B1': [('G1', 1, 0)],
    'B2': [('G2', 1, 0)],
    'B3': [('G3', 1, 0)],
    'B4': [('G4', 1, 0)],
}

original_graph = deep_copy_graph(graph)

def dijkstra(graph, start, goal):
    shortest_distance = {node: float('inf') for node in graph}
    shortest_distance[start] = 0
    predecessor = {}
    unvisited = set(graph.keys())
    while unvisited:
        current = min((node for node in unvisited), key=lambda n: shortest_distance[n])
        if current == goal:
            break
        unvisited.remove(current)
        for neighbor, cost, _ in graph[current]:
            distance = shortest_distance[current] + cost
            if distance < shortest_distance[neighbor]:
                shortest_distance[neighbor] = distance
                predecessor[neighbor] = current
    path = []
    current = goal
    while current != start:
        if current not in predecessor:
            return []
        path.insert(0, current)
        current = predecessor[current]
    path.insert(0, start)
    return path

def get_turn(current_node, next_node, robot_heading):
    for neighbor, _, node_to_node_direction in graph[current_node]:
        if neighbor == next_node:
            rel_turn = (node_to_node_direction - robot_heading) % 4
            return rel_turn , node_to_node_direction

put_box_nodes = ['G1', 'G2', 'G3', 'G4']  # Nodes where boxes can be put
take_box_nodes = ['S1', 'S2', 'S3', 'S4']  # Nodes where boxes can be taken

# Start navigation
goal_node = take_box_nodes[0]
start_node = put_box_nodes[0]

path = dijkstra(graph, start_node, goal_node)
path_index = 0
current_node = path[path_index]
next_node = path[path_index + 1]

ir = IRSensorArray()
imu = YawTracker()

Stop()
print("[PID-NODE+IMU] Following path:", path)

integral_line_error = 0.0
last_line_error = 0.0
last_loop_time = time.ticks_ms()
current_state = 'forward'

while True:
    ################# See ###################
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_loop_time) / 1000.0
    last_loop_time = current_time

    binary = ir.read_binary()
    Ultrasonic = get_distance()
    
#     obstacle_detected = Ultrasonic < 20.0 # Threshold for obstacle detection
    obstacle_detected = False
    
    weights = [-2, -1, 0, 1, 2]
    total_weighted_sum = 0
    black_sensors_count = 0
    for val, w in zip(binary, weights):
        if val == 0:
            total_weighted_sum += w
            black_sensors_count += 1
    err_line = (total_weighted_sum / black_sensors_count) if black_sensors_count else None

    print(f"Binary: {binary}, Err: {err_line}")
    
    if obstacle_detected:
        print("[ESP32] Obstacle detected, replanning...")

#         #wrobot_heading = (robot_heading - 2) % 4
        #Turn 180 degrees
        perform_turn(2,imu)
        
        blocked_node = next_node
        
        graph[current_node] = [entry for entry in graph[current_node] if entry[0] != blocked_node]
        graph[blocked_node] = [entry for entry in graph[blocked_node] if entry[0] != current_node]
        
        # Rerun Dijkstra
        path = dijkstra(graph, current_node, goal_node)
        
        if len(path) < 2:
            raise ValueError("No valid path after re-planning")
            
        path_index = 0
        next_node = path[1]
        current_node= path[0]
        just_replanned = True
        print("[ESP32] New path:", path)
        
        graph = deep_copy_graph(original_graph)
        
    if err_line is None:
        Stop()
        print("Line lost. Searching...")
        time.sleep(0.1)
        continue
    
    if binary.count(0) >= 3:
        print("Node detected")

        current_node = path[path_index]
        
        if current_node == goal_node and not took_box:
            print("Gonna take box")
            electromagnet.on()
            if robot_heading == 3:
                perform_turn(3,imu)
                
                # HERE?????????????????
            elif robot_heading == 0:
                imu_tracker.heading = 270
                perform_turn(2,imu)
            elif robot_heading == 1:
                perform_turn(1,imu)
            
            # Wait for a short time to simulate picking up the box
            line_follow_backward(1)
    
            # Replan path after picking up the box
            took_box = True
            current_node = "F"
            path = dijkstra(graph, current_node, put_box_nodes[0])  # Replan to the first put box node
            path_index = 0
            current_node = path[path_index]
            next_node = path[path_index + 1]
            #take_box_nodes.remove(current_node)  # Remove the node from take_box_nodes
            #print("[PID-NODE+IMU] Following path:", path)            current_state = 'forward'
            continue
        
        if current_node == put_box_nodes[0] and took_box:
            took_box = False
            #forward_bit()
            perform_turn(3,imu)
            electromagnet.off()
            while True:
                print("Reached goal node.")
                Stop()
                time.sleep(1)
        
        
        next_node = path[path_index + 1]
        rel_turn, new_heading = get_turn(current_node, next_node, robot_heading)
        path_index += 1
        print(f"[Move] From: {current_node}, To: {next_node}, Heading: {rel_turn}")

        perform_turn(rel_turn, imu)
        current_state = 'forward'

    if current_state == 'forward':
        proportional_term = KP_LINE * err_line
        integral_line_error += err_line * dt
        integral_line_error = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral_line_error))
        integral_term = KI_LINE * integral_line_error
        derivative_term = KD_LINE * ((err_line - last_line_error) / dt) if dt > 0 else 0.0
        last_line_error = err_line

        adjust = proportional_term + integral_term + derivative_term

        speed_r = BASE_SPEED - adjust
        speed_l = BASE_SPEED + adjust

        count_left = encoder_B.get_count()
        count_right = encoder_A.get_count()

# Speed correction based on encoder counts
        correction = (count_right - count_left) * 5  # Adjust gain as needed

# Apply correction to motor speeds
        speed_l_corrected = max(0, min(1023, int(speed_l - correction)))
        speed_r_corrected = max(0, min(1023, int(speed_r + correction)))

        set_speed(left_motor, speed_l_corrected)
        set_speed(left_motor_back, 0)
        set_speed(right_motor, speed_r_corrected)
        set_speed(right_motor_back, 0)

# Reset counts for next loop
        encoder_A.reset()
        encoder_B.reset()
    
    time.sleep(0.02)


