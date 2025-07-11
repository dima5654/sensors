from machine import Pin #For ESP32 to understand Pins
from time import sleep
from time import time
from tof_data import get_stable_reading, interpolate
from robot_motors import *
from odometry import Odometry
from HC-SR04 import *
from WaveShaker import *
#--- ALL variables for testing line-following ---


MAX_SPEED = 1023
speed = 0.4 * MAX_SPEED

stop_motors()

states = ['forward', 'forward_bit', 'swing_right', 'swing_left', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'

just_replanned = False

# --- Setup ---
button = Pin(39, Pin.IN, Pin.PULL_DOWN)
electromagnet = Pin(5, Pin.OUT)

left_encoder = Encoder(34, 35)
right_encoder = Encoder(32, 33)

odom = Odometry()

check = False
start_time = 0

# --- Graph definition ---
graph = {
    # Top row
    'A1': [('S1', 1, 2)], 
    'A2': [('S2', 1, 2)],
    'A3': [('S3', 1, 2)],
    'A4': [('S4', 1, 2)],
    
    # Entry lines
    'S1': [('A1', 1, 0), ('S2', 1, 1), ('F', 5, 2)],
    'S2': [('A2', 1, 0), ('S3', 1, 1), ('S1', 1, 3)],
    'S3': [('A3', 1, 0), ('S4', 1, 1), ('S2', 1, 3)],
    'S4': [('A4', 1, 0), ('L', 2, 1), ('S3', 1, 3)],
    
    #Right quarter
    'L': [('K', 5, 1), ('M', 3, 2), ('S4', 2, 3)],
    'K': [('J', 3, 2), ('L', 5, 3)],
    'M': [('L', 3, 0), ('J', 5, 1), ('H', 2, 2)],
    'J': [('K', 3, 0), ('I', 2, 2), ('M', 5, 3)],

    # Middle
    'F': [('S1', 5, 0), ('H', 5, 1), ('E', 2, 2)],
    'H': [('M', 2, 0), ('I', 5, 1), ('O', 2, 2), ('F', 5, 3)],
    'I': [('J', 2, 0), ('G4', 5, 2), ('H', 5, 3)],
    'E': [('F', 2, 0), ('O', 5, 1), ('D', 3, 2)],
    'O': [('H', 2, 0), ('C', 3, 2), ('E', 5, 3)],
    
    #Left quarter
    'D': [('E', 3, 0), ('C', 5, 1)],
    'C': [('O', 3, 0), ('G1', 2, 1), ('D', 5, 3)],

    # Entry lines
    'G1': [('G2', 1, 1), ('B1', 1, 2), ('C', 2, 3)],
    'G2': [('G3', 1, 1), ('B2', 1, 2), ('G1', 1, 3)],
    'G3': [('G4', 1, 1), ('B3', 1, 2), ('G2', 1, 3)],
    'G4': [('I', 5, 0), ('B4', 1, 2), ('G3', 1, 3)],
    
    #Bottom row
    'B1': [('G1', 1, 0)],
    'B2': [('G2', 1, 0)],
    'B3': [('G3', 1, 0)],
    'B4': [('G4', 1, 0)],
}

put_box_nodes = ['B1', 'B2', 'B3', 'B4']  # Nodes where boxes can be put
take_box_nodes = ['S1', 'S2', 'S3', 'S4']  # Nodes where boxes can be taken

goal_node = take_box_nodes[0]  # Example goal node (S1)
start_node = put_box_nodes[0]  # Example start node (B1)

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

# Get direction to neighbour node from Dijkstra path
def get_turn(current_node, next_node, robot_heading):
    for neighbor, cost, node_to_node_direction in graph[current_node]:
        if neighbor == next_node:
            rel_turn = (node_to_node_direction - robot_heading) % 4
            return rel_turn , node_to_node_direction
        
# --- Wait for button to be pressed ---
print("Press button to start")
while not button():
    sleep(0.1)

# --- Movement logic ---
robot_heading = 0  # 0: North, 1: East, 2: South, 3: West

#--- Path preparation ---
path = dijkstra(graph, start_node, goal_node)

path_index = 0
current_node = path[path_index]
next_node = path[path_index + 1]
print("[PATH FOUND]", path)

# --- Path for replanning if obstacle is detected---
full_path = []
full_path += path

# Main loop of Robot
while True:
    ##################   See   ###################
    # For delta_t
    end_time = time.ticks_ms()
    
    Ulrasonic = get_ultrasonic_filtered()
    WaveShaker = read_ir_sensors()

    raw = get_stable_reading()
    ToF = interpolate(raw)
    
    left_count = left_encoder.get_count()
    right_count = right_encoder.get_count()

    obstacle_detected = Ulrasonic < 15.0 # Threshold for obstacle detection

    left, center, right = WaveShaker[1] == '0', WaveShaker[2] == '0', WaveShaker[3] == '0'  # '0' means black line detected
    message = (left, center, right) # For debugging purposes

    ##################   Think   ###################
    
    #--- Replanning dijkstra if obstacle detected state ---

    if check == True:
        delta_t = start_time - end_time
        odom.update_encoders(left_count, right_count, delta_t)

    if obstacle_detected:
        print("[ESP32] Obstacle detected, replanning...")
        #Turn 180 degrees

        blocked_node = next_node
        
        graph[current_node] = [entry for entry in graph[current_node] if entry[0] != blocked_node]
        graph[blocked_node] = [entry for entry in graph[blocked_node] if entry[0] != current_node]
        
        robot_heading = (robot_heading - 2) % 4
        # Rerun Dijkstra
        path = dijkstra(graph, current_node, goal_node)
        
        for node in path:
            if node not in full_path:
                full_path.append(node)
            
        if len(path) < 2:
            raise ValueError("No valid path after re-planning")
            
        path_index = 0
        next_node = path[1]
        current_node= path[0]
        just_replanned = True
        print("[ESP32] New path:", path)

    #--- Picking box state ---
    if ToF < 20:  # Threshold for detecting a box mm

        print("[ESP32] Box detected by ToF sensor")
        #Move forward to pick up the box
        electromagnet.on()  
        # Wait for a short time to simulate picking up the box
        stop_motors()
        sleep(1)
        #Turn 180 degrees 
        robot_heading = (robot_heading - 2) % 4

    ##################   Line-following   ###################

    if left and center and right :
    # Only trigger once per node
        print("Node is detected")
            
        if just_replanned:
            just_replanned = False
        else:
            path_index += 1
            
        # Check if goal_node was reached
        take_box_nodes = ['S1', 'S2', 'S3', 'S4']  # Nodes where boxes can be taken

        if path_index + 1 >= len(path):
            print("The goal was reached")
            continue
            
        #Changing variables to new current node and next node
        
        current_node = path[path_index]
        next_node = path[path_index + 1]
            
        robot_heading, new_heading = get_turn(current_node, next_node, robot_heading)

        print(f"[Move] From: {current_node}, To: {next_node}, Heading: {new_heading}")
    if not right and center:
        current_state = 'swing_right'
       
    elif not left and center:
        current_state = 'swing_left' 

    if robot_heading == 0:
        current_state = 'forward'
    elif robot_heading == 1:
        current_state = 'turn_right'
    elif robot_heading == 2:
        current_state = 'turn_back'
    elif robot_heading == 3:
        current_state = 'turn_left'

    robot_heading = new_heading
    

    print(f"Sensor: {message.strip()} - State: {current_state}")

    ##################   Act   ###################
    if current_state == 'forward':
        left_motor_forward(speed) #leftMotor.setVelocity(speed)
        right_motor_forward(speed) #rightMotor.setVelocity(speed)

    if current_state == 'swing_right':
        left_motor_forward(speed * 0.5) #leftMotor.setVelocity(speed)
        right_motorforward(speed) #rightMotor.setVelocity(speed)
        
    if current_state == 'swing_left':
        left_motor_forward(speed * 0)
        right_motor_forward(speed * 0.5) #rightMotor.setVelocity(speed)
        
    if current_state == 'stop':
        stop_motors() 
        graph_data = data['graph_path']
        print("Final Path Taken:", ' -> '.join(graph_data))

    if current_state == 'forward_bit':
        turn_start = time.ticks_ms()
        while True:
            left_motor_forward(speed)
            right_motor_forward(speed) #rightMotor.setVelocity(speed)
            if turn_start >= 0.8:
                break
        current_state = 'forward'

    elif current_state == 'turn_right':
        turn_start = time.ticks_ms()
        while True:
            left_motor_forward(speed)
            right_motor_forward(speed) #rightMotor.setVelocity(speed)
            if turn_start >= 0.6:
                break
        turn_start = time.ticks_ms()
        while True:
            
            left_motor_forward(speed * 0.5)
            right_motor_backward(speed * 0.5) #rightMotor.setVelocity(speed)
            if turn_start >= 1.76:
                break
        current_state = 'forward'
        
    elif current_state == 'turn_left':
        turn_start = time.ticks_ms()
        while True:
        
            left_motor_forward(speed)
            right_motor_forward(speed) #rightMotor.setVelocity(speed)
            if turn_start >= 0.6:
                break
        turn_start = time.ticks_ms()
        while True:
    
            left_motor_backward(speed * 0.5)
            right_motor_forward(speed * 0.5) #rightMotor.setVelocity(speed)
            if turn_start >= 1.76:
                break
        current_state = 'forward'

        start_time = time.ticks_ms()

        if check == False:
            check = True
    
    sleep(0.01)
