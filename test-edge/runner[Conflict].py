#!/usr/bin/env python
"""
"""
from __future__ import absolute_import
from __future__ import print_function
from collections import defaultdict

import os
import sys
import optparse
import subprocess
import random

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(
        os.path.join(os.environ.get("SUMO_HOME"))
    )
    sys.path.append(
        '/opt/local/Library/Frameworks/Python.framework/Versions/2.7/lib/python2.7/site-packages/')
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME'")

import traci
# the port used for communicating with the sumo instance
PORT = 8873

#main execution loop to control each simulation step
def run():
    """execute the TraCI control loop"""
    traci.init(PORT)
    step = 0

    #get the names of each road/"edge"
    edge_IDs = traci.edge.getIDList()
    #get lane IDs
    lane_IDs = traci.lane.getIDList()
    #get list of vehicle IDS
    vehicle_IDs = traci.vehicle.getIDList()

    #need to work out priority lanes before simulation to ensure they don't get over-writen
    global_priority_lanes = defaultdict(list)
    for edge in edge_IDs:
        edge_lanes = [lane for lane in lane_IDs if traci.lane.getEdgeID(lane) == edge]
        global_priority_lanes[edge].append(get_priority_lanes(edge_lanes))

    while traci.simulation.getMinExpectedNumber() > 0:

        #simulate congested road network until a valid congestion value can be figure out
        for edge in edge_IDs:
            #lanes
            edge_lanes = [lane for lane in lane_IDs if traci.lane.getEdgeID(lane) == edge]
            priority_lanes = global_priority_lanes[edge][0] #hack to retrieve the actual list from the dict
            non_priority_lanes = set(edge_lanes) - set(priority_lanes)
            #vehicles
            edge_vehicles = traci.edge.getLastStepVehicleIDs(edge)
            #perform cognitive radio steps
            simulate_congestion(non_priority_lanes, 2)
            enable_priority_access(priority_lanes)
            update_edge_travel_time(edge)
            update_vehicle_travel_time(edge_vehicles)

        traci.simulationStep()
        step += 1
    traci.close()
    sys.stdout.flush() 

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
    """Disables standard vehicles defined in 'priority access' to drive in priority lanes"""
    priority_access = set(['passenger', 'private', 'evehicle'])
    for lane in priority_lanes:
        currently_allowed = set(traci.lane.getAllowed(lane))
        traci.lane.setAllowed(lane, list(currently_allowed - priority_access))

def update_edge_travel_time(edge):
    """Updates the travel time on given edge"""
    traci.edge.adaptTraveltime(edge, traci.edge.getTraveltime(edge))

def update_vehicle_travel_time(vehicles):
    """Updates the travel time on each route"""
    for vehicle in vehicles:
        traci.vehicle.rerouteTraveltime(vehicle)

def simulate_congestion(lanes, speed):
    for lane in lanes:
        traci.lane.setMaxSpeed(lane, speed)


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pWE = 1. / 10
    pEW = 1. / 10
    with open("single-edge-generated.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="bus" vClass="bus" accel="0.5" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="25.00" guiShape="bus"/>
        <vType id="car" vClass="private" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="30.00" lcKeepRight="0" guiShape="passenger/hatchback"/>
        <vType id="priority" vClass="ignoring" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="30.00" lcKeepRight="0" guiShape="passenger/hatchback"/>

        <route id="WE" edges="anti-clock-1 anti-clock-2" />
        <route id="EW" edges="clock-wise-2 clock-wise-1" />""", file=routes)
        lastVeh = 0
        vehNr = 0
        busLane = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="WE_%i" type="car" route="WE" depart="%i" departLane="%i" />' % (
                    vehNr, i, random.randint(1,2)), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="EW_%i" type="car" route="EW" depart="%i" departLane="%i" />' % (
                    vehNr, i, random.randint(1,2)), file=routes)
                vehNr += 1
                lastVeh = i
            # if i % 10 == 0:
            #     if random.uniform(0, 1) < pWE:
            #         print('    <vehicle id="WE_%i" type="bus" route="WE" depart="%i" departLane="%i" />' % (
            #             vehNr, i, busLane), file=routes)
            #         vehNr += 1
            #         lastVeh = i
            #     if random.uniform(0, 1) < pEW:
            #         print('    <vehicle id="EW_%i" type="bus" route="EW" depart="%i" departLane="%i" />' % (
            #             vehNr, i, busLane), file=routes)
            #         vehNr += 1
            #         lastVeh = i

        print("</routes>", file=routes)


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

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "1-long-road.sumocfg", "--tripinfo-output",
                                    "tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
