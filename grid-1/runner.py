#!/usr/bin/env python
"""
"""
from __future__ import absolute_import
from __future__ import print_function
from collections import defaultdict

import os
import sys
import optparse

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(
        os.path.join(os.environ.get("SUMO_HOME"))
    )
    sys.path.append(
        '/opt/local/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages/')
    import sumolib
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME'")

import traci
# the port used for communicating with the sumo instance

#main execution loop to control each simulation step


def run():
    """execute the TraCI control loop"""
    step = 0

    #parse the network file to improve performance when retrieving lanes
    NET_FILE = 'grid-1.net.xml'
    NET = sumolib.net.readNet(NET_FILE)
    #get the names of each road/"edge"
    EDGE_IDS = NET.getEdges()

    # need to work out priority lanes before simulation to ensure they don't
    # get over-writen
    global_edge_lanes = defaultdict(list)
    global_priority_lanes = defaultdict(list)
    for edge in EDGE_IDS:
        edge_lanes = [ln.getID() for ln in sumolib.net.Edge.getLanes(edge)]
        global_edge_lanes[edge.getID()].append(edge_lanes)
        global_priority_lanes[edge.getID()].append(get_priority_lanes(edge_lanes))

    while traci.simulation.getMinExpectedNumber() > 0:
        # simulate congested road network until a valid congestion value can be
        # figure out
        for edge in EDGE_IDS:
            #lanes
            edge_lanes = global_edge_lanes[edge.getID()][0]
            priority_lanes = global_priority_lanes[edge.getID()][0]# hack to retrieve the actual list from the dict
            non_priority_lanes = set(edge_lanes) - set(priority_lanes)
            #vehicles
            edge_vehicles = traci.edge.getLastStepVehicleIDs(edge.getID())
            #perform cognitive radio steps
            if detect_priority_vehicle(priority_lanes):
                disable_priority_access(priority_lanes)
                clean_priority_lanes(priority_lanes)
                update_edge_travel_time(edge.getID())
                update_vehicle_travel_time(edge_vehicles)
            else:
                # simulate_congestion(non_priority_lanes, 2)
                enable_priority_access(priority_lanes)
                update_edge_travel_time(edge.getID())
                update_vehicle_travel_time(edge_vehicles)
        traci.simulationStep()
        step += 1
    traci.close()
    sys.stdout.flush()

def detect_priority_vehicle(priority_lanes):
    priority_types = set(['bus', 'emergency', 'taxi'])
    for lane in priority_lanes:
        vehicles = traci.lane.getLastStepVehicleIDs(lane)
        for vehicle in vehicles:
            if(traci.vehicle.getVehicleClass(vehicle) in priority_types):
                return True
    return False

def get_priority_lanes(lanes):
    """Takes a list of lanes and returns priority lanes"""
    priority_lanes = []
    priority_types = set(['bus', 'emergency', 'taxi'])
    non_priority_types = set(['private', 'evehicle', 'passenger', 'truck'])
    for lane in lanes:
        lane_types = set(traci.lane.getAllowed(lane))
        if lane_types & priority_types and not lane_types & non_priority_types:
            priority_lanes.append(lane)
    return priority_lanes

def enable_priority_access(priority_lanes):
    """Enables standard vehicles defined in 'priority access' to drive in priority lanes"""
    priority_access = set(['passenger', 'private', 'evehicle'])
    for lane in priority_lanes:
        currently_allowed = set(traci.lane.getAllowed(lane))
        traci.lane.setAllowed(lane, list(currently_allowed | priority_access))

def disable_priority_access(priority_lanes):
    """Stops non-priority vehicles driving on priority lanes, 
    attempts to turn off successive priority lanes in order to avoid congestion for the priority vehicle"""
    priority_access = set(['passenger', 'private', 'evehicle'])
    for lane in priority_lanes:
        currently_allowed = set(traci.lane.getAllowed(lane))
        # connected_priority_lanes = traci.lane.getLinks(lane)
        traci.lane.setAllowed(lane, list(currently_allowed - priority_access))
        # if len(connected_priority_lanes) > 0:
        #     for connected_priority_lane in connected_priority_lanes:
        #         currently_allowed = set(traci.lane.getAllowed(connected_priority_lane[0]))
        #         traci.lane.setAllowed(connected_priority_lane[0], list(currently_allowed - priority_access))

def clean_priority_lanes(priority_lanes):
    priority_types = set(['bus', 'emergency', 'taxi'])
    for lane in priority_lanes:
        lane_id = get_lane_id(lane)
        current_vehicles = traci.lane.getLastStepVehicleIDs(lane)
        for vehicle in current_vehicles:
            if traci.vehicle.getVehicleClass not in priority_types:
                traci.vehicle.changeLane(vehicle, lane_id+1, 0)

def update_edge_travel_time(edge):
    """Updates the travel time on given edge"""
    traci.edge.adaptTraveltime(edge, traci.edge.getTraveltime(edge))

def update_vehicle_travel_time(vehicles):
    """Updates the travel time on each route"""
    for vehicle in vehicles:
        traci.vehicle.rerouteTraveltime(vehicle)

def simulate_congestion(lanes, speed):
    """Reduces speed for all given lanes to simulate congestion"""
    for lane in lanes:
        traci.lane.setMaxSpeed(lane, speed)

def get_lane_id(lane_ID):
    """Returns the sumo appended index from the lane ID"""
    return int(lane_ID.split('_')[1])

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    # sumoProcess = subprocess.Popen([sumoBinary, "-c", "grid-1.sumocfg", "--tripinfo-output",
    #                                 "tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    traci.start(["sumo-gui", "-c", "grid-1.sumocfg"]) 
    run()
    # sumoProcess.wait()
