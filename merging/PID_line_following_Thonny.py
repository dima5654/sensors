from machine import Pin, PWM
import time
from WaveShaker import *

# Motor setup
freq = 1000
left_motor = PWM(Pin(14), freq=freq)
left_motor_back = PWM(Pin(12), freq=freq)
right_motor = PWM(Pin(27), freq=freq)
right_motor_back = PWM(Pin(26), freq=freq)

# --- PID Tuning Constants ---
KP_LINE         = 80.0
KI_LINE         = 0.5
KD_LINE         = 500.0
INTEGRAL_MAX    = 100
BASE_SPEED      = 700

# --- Movement logic ---
robot_heading = 0

def set_speed(pwm, duty):
    pwm.duty(duty)

def forward(vel=500): 
    set_speed(left_motor, vel)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, vel)
    set_speed(right_motor_back, 0)

def Stop():
    set_speed(left_motor, 0)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, 0)

# --- Graph for Dijkstra ---
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

# --- Path Planning ---
goal_node = 'S1'
start_node = 'G1'
path = dijkstra(graph, start_node, goal_node)
path_index = 0
current_node = path[path_index]
next_node = path[path_index + 1]

ir = IRSensorArray()
Stop()
print("[PID-NODE] Following path:", path)

integral_line_error = 0.0
last_line_error = 0.0
last_loop_time = time.ticks_ms()
current_state = 'forward'

while True:
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_loop_time) / 1000.0
    last_loop_time = current_time

    binary = ir.read_binary()
    weights = [-2, -1, 0, 1, 2]
    total_weighted_sum = 0
    black_sensors_count = 0
    for val, w in zip(binary, weights):
        if val == 0:
            total_weighted_sum += w
            black_sensors_count += 1
    err_line = (total_weighted_sum / black_sensors_count) if black_sensors_count else None

    print(f"Binary: {binary}, Err: {err_line}")

    if err_line is None:
        Stop()
        print("Line lost. Searching...")
        time.sleep(0.1)
        continue

    if binary.count(0) >= 3:
        print("Node detected")
        if path_index == len(path) - 1:
            print("Reached goal node.")
            Stop()
            while True:
                time.sleep(1)

        current_node = path[path_index]
        next_node = path[path_index + 1]
        rel_turn, new_heading = get_turn(current_node, next_node, robot_heading)
        path_index += 1

        if rel_turn == 0:
            print("Moving forward bit to clear node")
            forward(BASE_SPEED)
            time.sleep(0.5)
            current_state = 'forward'
        elif rel_turn == 1:
            print("Turning right")
            forward(BASE_SPEED)
            time.sleep(0.2)
            set_speed(left_motor, BASE_SPEED)
            set_speed(left_motor_back, 0)
            set_speed(right_motor, 0)
            set_speed(right_motor_back, BASE_SPEED)
            time.sleep(1.2)
            current_state = 'forward'
        elif rel_turn == 3:
            print("Turning left")
            forward(BASE_SPEED)
            time.sleep(0.2)
            set_speed(left_motor, 0)
            set_speed(left_motor_back, BASE_SPEED)
            set_speed(right_motor, BASE_SPEED)
            set_speed(right_motor_back, 0)
            time.sleep(1)
            current_state = 'forward'

        robot_heading = new_heading

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

        set_speed(left_motor, max(0, min(1023, int(speed_l))))
        set_speed(left_motor_back, 0)
        set_speed(right_motor, max(0, min(1023, int(speed_r))))
        set_speed(right_motor_back, 0)

    time.sleep(0.02)

