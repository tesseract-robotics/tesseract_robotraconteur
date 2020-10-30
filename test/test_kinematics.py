from RobotRaconteur.Client import *
import numpy as np
import argparse

parser = argparse.ArgumentParser(description="Basic sanity test for planner")
parser.add_argument("--url",type=str,help="URL of planner service",default='rr+tcp://localhost:63158?service=tesseract')
parser.add_argument("--plot",default=False,action="store_true",help="Plot results")
args, _ = parser.parse_known_args()

c = RRN.ConnectService(args.url)

Identifier = RRN.GetStructureType('com.robotraconteur.identifier.Identifier',c)
robot_id = Identifier()
robot_id.name = "manipulator"
uuid_dt = RRN.GetNamedArrayDType('com.robotraconteur.uuid.UUID',c)
robot_id.uuid=np.zeros((1,), uuid_dt)

for _ in range(100):
    joint_angles = np.random.rand(7)
    tcp_pose = c.fwdkin(robot_id,joint_angles,None)
    
    seed = np.random.rand(7)
    inv_kin_result = c.invkin(robot_id, tcp_pose, seed)
    
    assert(len(inv_kin_result) > 0)
    tcp_pose2 = c.fwdkin(robot_id,inv_kin_result[0].joints[0],None)

    tcp_pose_a = RRN.NamedArrayToArray(tcp_pose.pose)
    tcp_pose2_a = RRN.NamedArrayToArray(tcp_pose2.pose)

    np.testing.assert_allclose(tcp_pose_a,tcp_pose2_a,atol=1e-3)