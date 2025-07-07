from machine import Pin, PWM
import time
from WaveShaker import *

# Motor setup
freq = 1000
left_motor = PWM(Pin(14), freq=freq)
left_motor_back = PWM(Pin(12), freq=freq)
right_motor = PWM(Pin(27), freq=freq)
right_motor_back = PWM(Pin(26), freq=freq)

# --- Movement logic ---
robot_heading = 0  # 0: North, 1: East, 2: South, 3: West
#gs_sensor_pin = [
 #   Pin(15, Pin.IN),  # Left
#   #  Pin(2, Pin.IN),  # Left mid
   # Pin(4, Pin.IN),  # Middle
    #Pin(16, Pin.IN),  # Right mid
    #Pin(17, Pin.IN)   # Right
#]

def set_speed(pwm, duty):
    pwm.duty(duty)

def forward(vel=500): 
    set_speed(left_motor, vel)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, vel)
    set_speed(right_motor_back, 0)

def left(vel=500):
    set_speed(left_motor, 0)
    set_speed(left_motor_back, vel)
    set_speed(right_motor, vel)
    set_speed(right_motor_back, 0)

def right(vel=500):
    set_speed(left_motor, vel)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, vel)

def Stop():
    set_speed(left_motor, 0)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, 0)

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
    for neighbor, cost, node_to_node_direction in graph[current_node]:
        if neighbor == next_node:
            rel_turn = (node_to_node_direction - robot_heading) % 4
            return rel_turn , node_to_node_direction
        
#--- Path preparation ---
goal_node = 'S1'  # Example goal node (E)
start_node = 'G1'  # Example start node (G1)

states = ['forward', 'forward_bit', 'swing_right', 'swing_left', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'



path = dijkstra(graph, start_node, goal_node)

path_index = 0
current_node = path[path_index]
next_node = path[path_index + 1]
print("[PATH FOUND]", path)
ir = IRSensorArray()
Stop()
# Main loop for line following
while True:

    ##################   See   ###################

    sensors = ir.read_binary()

    far_left, left_val, center, right_val, far_right = sensors[0], sensors[1], sensors[2], sensors[3], sensors[4]  # '0' means black line detected
    message = (far_left, left_val, center, right_val, far_right) # For debugging purposes
    ################## Think ###################

    
        
    print(left_val, left_val, center, right_val, right_val)

    if center == 0 or (right_val == 0 and far_right == 1) or (left_val == 0 and far_left == 1):
        current_state = 'forward'
    elif center == 1 and (left_val == 0 or far_left == 0):
        current_state = 'swing_left'
    elif center == 1 and (right_val == 0 or far_right == 0):
        current_state = 'swing_right'
    else:
        Stop()
        
    if [far_left, left_val, center, right_val, far_right].count(0) >= 3:
    # Only trigger once per node
        print("Node is detected")
            
        #if just_replanned:
         #   just_replanned = False
        #else:
         #   
            
        #Changing variables to new current node and next node
        if path[path_index] == goal_node:
            print("Goal reached!")
            Stop()
            while True:
                time.sleep(1)
        
        current_node = path[path_index]
        next_node = path[path_index + 1]
        rel_turn, new_heading = get_turn(current_node, next_node, robot_heading)
        path_index += 1
        
        print(f"[Move] From: {current_node}, To: {next_node}, Heading: {rel_turn}")
        if rel_turn == 0:
            current_state = 'forward_bit'
        elif rel_turn == 1:
            current_state = 'turn_right'
        elif rel_turn == 2:
            current_state = 'turn_back'
        elif rel_turn == 3:
            current_state = 'turn_left'
        robot_heading = new_heading
            
################## Act ###################
    if current_state == 'forward':
        forward(700)
    elif current_state == 'swing_left':
        print("Swinging left")
        left(500)
    elif current_state == 'swing_right':
        print("Swinging right")
        right(500)
    elif current_state == 'turn_left':
        print("Going left")
        forward(500)
        time.sleep(0.5)
        left(600)
        time.sleep(1)
        
        current_state = 'forward'
    elif current_state == 'turn_right':
        print("Going right")
        forward(500)
        time.sleep(0.5)
        right(600)
        time.sleep(1)

        current_state = 'forward'
    elif current_state == 'forward_bit':
        forward(500)
        print("Going bit_forward")
        time.sleep(0.5)
        current_state = 'forward'
        
    time.sleep(0.05)
