
import ujson as json
from machine import Pin #For ESP32 to understand Pins
from time import sleep
import time
#--- ALL variables for testing line-following ---

counter = 0
COUNTER_MAX = 5
count_if_node = 0
state_updated = False

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED

leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

states = ['forward', 'forward_bit', 'swing_right', 'swing_left', 'turn_right', 'turn_left', 'turn_back', 'stop','nothing']
current_state = 'forward'

loop_counter = 0
just_replanned = False
graph_path = []

# Variables to implement the work of robot turning
node_detected = False
webots_not_busy = True
robot_heading = 0
# --- Button setup ---
button = Pin(39, Pin.IN, Pin.PULL_DOWN)

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
goal_node = 'B4'  # Example goal node
start_node = 'A1'  # Example start node

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
path_index = 0
current_node = path[path_index]
next_node = path[path_index + 1]
path = dijkstra(graph, data['start'], goal_node)
print("[PATH FOUND]", path)
graph_path += path


while True:
    
    ##################   See   ###################
    start = time.ticks_ms()
     
    Ulrasonic = get_ultrasonic_filtered()
    WaveShaker = read_ir_sensors()

    raw = get_stable_reading()
    ToF = interpolate(raw)

    obstacle_detected = Ulrasonic < 20.0 # Threshold for obstacle detection

    left, center, right = WaveShaker[0] == '0', WaveShaker[1] == '0', WaveShaker[2] == '0'  # '0' means black line detected


    ##################   Think   ###################
    if obstacle_detected:
        print("[ESP32] Obstacle detected, replanning...")
        #Turn 180 degrees


        blocked_node = next_node
        
        graph[current_node] = [entry for entry in graph[current_node] if entry[0] != blocked_node]
        graph[blocked_node] = [entry for entry in graph[blocked_node] if entry[0] != current_node]
        
        robot_heading = (robot_heading - 2) % 4
        
        # Rerun Dijkstra
        try:
            path = dijkstra(graph, current_node, goal_node)
            for node in path:
                if node not in graph_path:
                    graph_path.append(node)
            
            if len(path) < 2:
                raise ValueError("No valid path after re-planning")
            path_index = 0
            next_node = path[1]
            current_node= path[0]
            node_detected = False
            just_replanned = True
            print("[ESP32] New path:", path)
        except Exception as e:
            print("[ESP32] Replanning failed:", str(e))
            conn.send(b'fail\n')  # Inform Webots that replanning failed
            continue  # Skip the rest of this loop
     
    if left and center and right :
        if not node_detected:  # Only trigger once per node
            print("Node is detected")
            node_detected = True
            
            if just_replanned:
                just_replanned = False
            else:
                path_index += 1
            
            # Check if goal_node was reached
        if path_index + 1 >= len(path):
            conn.send(b'stop\n')
            print("The goal was reached")
                
            try:
                msg = json.dumps({'graph_path': graph_path}) + '\n'
                conn.send(msg.encode())
            except Exception as e:
                print("Failed to send final path:", e)
            continue
            
        #Changing variables to new current node and next node
        
        current_node = path[path_index]
        next_node = path[path_index + 1]
            
        rel_turn, new_heading = get_turn(current_node, next_node, robot_heading)

        print(f"[Move] From: {current_node}, To: {next_node}, Heading: {new_heading}")
            
            ##################   Act   ###################
            
                # Send new state
        if rel_turn == 0:
            conn.send(b'forward_bit\n')
        elif rel_turn == 1:
            conn.send(b'turn_right\n')
        elif rel_turn == 2:
            conn.send(b'turn_back\n')
        elif rel_turn == 3:
            conn.send(b'turn_left\n')
        robot_heading = new_heading
        webots_not_busy = False
                # Should wait for Webots finishing turning and going forward a bit that sensor get not 111
    elif not (left and center and right):
        node_detected = False
        conn.send(b'nothing\n')
        
    end = time.ticks_ms()
    loop_counter += 1
    if loop_counter % 20 == 0:
        print("Loop time:", time.ticks_diff(end, start), "ms")
    sleep(0.01)


