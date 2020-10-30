from RobotRaconteur.Client import *
import numpy as np
import argparse
import time

parser = argparse.ArgumentParser(description="Basic sanity test for contacts")
parser.add_argument("--url",type=str,help="URL of planner service",default='rr+tcp://localhost:63158?service=tesseract')
args, _ = parser.parse_known_args()

c = RRN.ConnectService(args.url)

planning_constants = RRN.GetConstants("com.robotraconteur.robotics.planning",c)
contact_test_type = planning_constants["ContactTestTypeCode"]

joint_names = [f"joint_a{i+1}" for i in range(7)]
joint_pos = np.zeros((7,),dtype=np.float)

PlannerJointPositions = RRN.GetStructureType('com.robotraconteur.robotics.planning.PlannerJointPositions',c)
joint_positions = PlannerJointPositions()
joint_positions.joint_position = { k: np.array(v) for k,v in zip(joint_names,joint_pos) }

contacts = c.compute_contacts(joint_positions, contact_test_type["all"], 0.1)

assert(len(contacts) == 0)

#Move robot into self collision
joint_positions.joint_position["joint_a2"] = 1.8
joint_positions.joint_position["joint_a4"] = -2.1
joint_positions.joint_position["joint_a6"] = 1.2

t1 = time.time()
contacts = c.compute_contacts(joint_positions, contact_test_type["all"], 0.1)
print(f"time: {time.time()-t1}")

assert len(contacts) == 3