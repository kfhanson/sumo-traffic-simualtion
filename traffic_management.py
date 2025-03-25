import os
import sys
import traci
import time
from sumolib import net
import pandas as pd
import xml.etree.ElementTree as ET

try:
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
        from sumolib import checkBinary
    else:
        sys.exit("SUMO_HOME not found!")
except ImportError as e:
    sys.exit(f"Error importing SUMO tools: {e}")

sumo_binary = checkBinary('sumo-gui')
sumo_config = "osm.sumocfg"
route = "bsdroute.rou.xml"
net_file = "osm.net.xml"

traffic_light_id = 'joinedS_6421159832_cluster_3639980474_3640024452_3640024453_6421159827_#4more_cluster_6421159831_7129012339'
congestion_threshold = 5
adjustment_step = 5
max_green_duration = 50
min_green_duration = 13

def get_approach_mapping(net_obj):
    approach_mapping = {}
    edges_with_detectors = [
        "754598165#2", # North
        "1053267667#3", # South
        "749662140#0", # East
        "885403818#2", # West
    ]

    for edge_id in edges_with_detectors:
        edge = net_obj.getEdge(edge_id)
        if edge is None:
            print(f"Warning: Edge '{edge_id}' not found in network.")
            continue 
        if edge.getFunction() != 'internal':
            for lane in edge.getLanes():
                lane_id = lane.getID()
                if "754598165#2" in lane_id:
                    approach_mapping[lane_id] = "north"
                elif "1053267667#3" in lane_id:
                    approach_mapping[lane_id] = "south"
                elif "749662140#0" in lane_id:
                    approach_mapping[lane_id] = "east"
                elif "885403818#0" in lane_id:
                    approach_mapping[lane_id] = "west"
    return approach_mapping

def create_additional_file(filename, net_file):
    net_obj = net.readNet(net_file)
    with open(filename, "w") as f:
        f.write('<additional>\n')
        edges_with_detectors = [
            "754598165#2", # North
            "1053267667#3", # South
            "749662140#0", # East
            "885403818#2", # West
        ]
        for edge_id in edges_with_detectors:
            edge = net_obj.getEdge(edge_id)
            if edge is None:
                print(f"Warning: Edge '{edge_id}' not found in network.")
                continue
            if edge.getFunction() != 'internal':
                for lane in edge.getLanes():
                    lane_id = lane.getID()
                    length = lane.getLength()
                    position = length - 5
                    if position < 0:
                        position = 0.1
                    detector_id = f"detector_{lane_id}"
                    f.write(f'  <inductionLoop id="{detector_id}" lane="{lane_id}" pos="{position:.2f}" freq="1" file="detector_output.xml"/>\n')
        f.write('</additional>\n')

def run_simulation():
    additional_file = "detectors.add.xml"
    create_additional_file(additional_file, net_file)
    
    sumo_cmd = [
        sumo_binary, "-c", sumo_config, 
        "--additional-files", additional_file, 
        "--tripinfo-output", "tripinfo.xml", 
        "--summary-output", "summary.xml", 
        "--no-step-log", "true", 
        "--time-to-teleport", "-1"
        ]
    traci.start(sumo_cmd)
    net_obj = net.readNet(net_file)
    approach_mapping = get_approach_mapping(net_obj)

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            vehicle_counts = {"north": 0, "south": 0, "east": 0, "west": 0}
            for edge in net_obj.getEdges():
                if edge.getFunction() != 'internal':
                    for lane in edge.getLanes():
                        lane_id = lane.getID()
                        detector_id = f"detector_{lane_id}"
                        if lane_id in approach_mapping:
                            approach = approach_mapping[lane_id]
                            vehicle_counts[approach] += traci.inductionloop.getLastStepVehicleNumber(detector_id)
                            
            congested_approaches = [approach for approach, count in vehicle_counts.items() if count > congestion_threshold]
            current_logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(traffic_light_id)[0]
            current_phase_index = traci.trafficlight.getPhase(traffic_light_id)

            if (traci.trafficlight.getNextSwitch(traffic_light_id) - traci.simulation.getTime()) <= 1:
                if "G" in current_logic.phases[current_phase_index].state:
                    if len(congested_approaches) > 0:
                        current_logic.phases[current_phase_index].duration = min(current_logic.phases[current_phase_index].duration + adjustment_step, max_green_duration)
                for i, phase in enumerate(current_logic.phases):
                    if i != current_phase_index and "G" in phase.state:
                        if len(congested_approaches) > 0:
                            current_logic.phases[i].duration = max(current_logic.phases[i].duration - adjustment_step, min_green_duration)
                traci.trafficlight.setProgramLogic(traffic_light_id, current_logic)
            print(f"Time: {traci.simulation.getTime()}, Vehicle Counts: {vehicle_counts}, Congested: {congested_approaches}, Durations: {[phase.duration for phase in current_logic.phases]}")
            time.sleep(0.1)
    except traci.exceptions.FatalTraCIError as e:
        print(f"Error: {e}")
    finally:
        traci.close()

def analyze_tripinfo(tripinfo_file):
    try:
        tree = ET.parse(tripinfo_file)
        root = tree.getroot()

        data = []
        for tripinfo in root.findall('tripinfo'):
            trip_data = {
                'id': tripinfo.get('id'),
                'depart': float(tripinfo.get('depart')),
                'arrival': float(tripinfo.get('arrival')),
                'duration': float(tripinfo.get('duration')),
                'waitingTime': float(tripinfo.get('waitingTime')),
                'timeLoss': float(tripinfo.get('timeLoss')),
            }
            data.append(trip_data)

        df = pd.DataFrame(data)

        print("Average trip duration:", df['duration'].mean())
        print("Average waiting time:", df['waitingTime'].mean())
        print("Average time loss:", df['timeLoss'].mean())
        print(df.describe())

    except FileNotFoundError:
        print(f"Error: File '{tripinfo_file}' not found.  Make sure the simulation ran and generated the file.")
    except ET.ParseError:
        print(f"Error: Could not parse '{tripinfo_file}'.  It might be an invalid XML file.")

if __name__ == "__main__":
    run_simulation()
    analyze_tripinfo("tripinfo.xml")