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

sumo_binary = "sumo-gui"
sumo_config = "osm.sumocfg"
route = "bsdroute.xml"
net_file = "osm.net.xml"

sensor_ids = {
    "north": "sensor_north",
    "south": "sensor_south",
    "east": "sensor_east",
    "west": "sensor_west",
}

traffic_light_id = 'joinedS_6421159832_cluster_3639980474_3640024452_3640024453_6421159827_#4more_cluster_6421159831_7129012339'

def create_additional_file(filename, sensor_ids, net_file):
    net_obj = net.readNet(net_file)
    with open(filename, "w") as f:
        f.write('<additional>\n')
        for edge in net_obj.getEdges():
            if edge.getFunction() != 'internal':
                for lane in edge.getLanes():
                    lane_id = lane.getID()
                    length = lane.getLength()
                    position = length-5
                    if position < 0:
                        position = 0.1
                    detector_id = f"detector_{lane_id}"
                    f.write(f'  <inductionLoop id="{detector_id}" lane="{lane_id}" pos="{position}" freq="1" file="detector_output.xml"/>\n')
        f.write('</additional>\n')

def run_simulation():
    additional_file = "detectors.add.xml"
    create_additional_file(additional_file, sensor_ids, net_file)
    
    sumo_cmd = [sumo_binary, "-c", sumo_config, "--additional-files", additional_file, "--tripinfo-output", "tripinfo.xml", "--summary-output", "summary.xml", "--no-step-log", "true", "--time-to-teleport", "-1"]
    traci.start(sumo_cmd)
    net_obj = net.readNet(net_file)

    default_durations = [12, 6]
    congestion_threshold = 5
    adjustment_step = 5

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicle_counts = {}
        for approach, _ in sensor_ids.items():
            vehicle_counts[approach] = 0
        for edge in net_obj.getEdges():
            if edge.getFunction() != 'internal':
                for lane in edge.getLanes():
                    lane_id = lane.getID()
                    detector_id = f"detector_{lane_id}"
                    for approach in sensor_ids:
                        if approach in lane_id:
                            vehicle_counts[approach] += traci.inductionloop.getLastStepVehicleNumber(detector_id)
                            break

        congested_approaches = [approach for approach, count in vehicle_counts.items() if count > congestion_threshold]
        current_state = traci.trafficlight.getRedYellowGreenState(traffic_light_id)
        current_logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(traffic_light_id)[0]

        if len(congested_approaches) > 0:
            if ("north" in congested_approaches) or ("south" in congested_approaches):
                if (traci.trafficlight.getNextSwitch(traffic_light_id) - traci.simulation.getTime()) <= 1:
                    current_logic.phases[0].duration = min(current_logic.phases[0].duration + adjustment_step, 50)
                    if len(current_logic.phases) > 2:
                        current_logic.phases[2].duration = max(current_logic.phases[2].duration - adjustment_step, 13)
                    if len(current_logic.phases) > 4:
                        current_logic.phases[4].duration = max(current_logic.phases[4].duration - adjustment_step, 13)
                    traci.trafficlight.setProgramLogic(traffic_light_id, current_logic)

            if ("east" in congested_approaches) or ("west" in congested_approaches):
                if (traci.trafficlight.getNextSwitch(traffic_light_id) - traci.simulation.getTime()) <= 1:
                    if len(current_logic.phases) > 2:
                        current_logic.phases[2].duration = min(current_logic.phases[2].duration + adjustment_step, 50)
                    current_logic.phases[0].duration = max(current_logic.phases[0].duration - adjustment_step, 13)
                    if len(current_logic.phases) > 4: 
                        current_logic.phases[4].duration = max(current_logic.phases[4].duration - adjustment_step, 13)
                    traci.trafficlight.setProgramLogic(traffic_light_id, current_logic)

            if any(approach in congested_approaches for approach in ["east", "west", "north", "south"]):
                if (traci.trafficlight.getNextSwitch(traffic_light_id) - traci.simulation.getTime()) <= 1:
                    if len(current_logic.phases) > 4:
                        current_logic.phases[4].duration = min(current_logic.phases[4].duration + adjustment_step, 50)
                    current_logic.phases[0].duration = max(current_logic.phases[0].duration - adjustment_step, 13)
                    if len(current_logic.phases) > 2:
                        current_logic.phases[2].duration = max(current_logic.phases[2].duration - adjustment_step, 13)
                    traci.trafficlight.setProgramLogic(traffic_light_id, current_logic)


        print(f"Time: {traci.simulation.getTime()}, Vehicle Counts: {vehicle_counts}, Congested: {congested_approaches}, Current State: {current_state}, Durations: {[phase.duration for phase in current_logic.phases]}")
        time.sleep(0.1)

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