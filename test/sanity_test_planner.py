from RobotRaconteur.Client import *
import numpy as np

c = RRN.ConnectService('rr+tcp://localhost:63158?service=tesseract')

start_waypoint = RRN.NewStructure('com.robotraconteur.robotics.planning.JointWaypoint',c)
end_waypoint = RRN.NewStructure('com.robotraconteur.robotics.planning.JointWaypoint',c)

start_waypoint.joint_positions = np.ones((7,))*.1
start_waypoint.coeffs = np.ones((1,))
start_waypoint.is_critical=True

end_waypoint.joint_positions = np.ones((7,))*.01
end_waypoint.coeffs = np.ones((1,))
end_waypoint.is_critical=True

planning_request = RRN.NewStructure('com.robotraconteur.robotics.planning.PlanningRequest',c)
planning_request.device = RRN.NewStructure('com.robotraconteur.identifier.Identifier',c)
planning_request.device.name = "right_arm"
uuid_dt = RRN.GetNamedArrayDType('com.robotraconteur.uuid.UUID',c)
planning_request.device.uuid=np.zeros((1,), uuid_dt)
box_dt = RRN.GetNamedArrayDType('com.robotraconteur.geometry.Box',c)
bounds = np.zeros((1,),box_dt)
bounds[0]["origin"]["x"] = -10
bounds[0]["origin"]["y"] = -10
bounds[0]["origin"]["z"] = -10
bounds[0]["size"]["width"] = 20
bounds[0]["size"]["height"] = 20
bounds[0]["size"]["depth"] = 20
planning_request.workspace_bounds = bounds
planning_request.collision_check=True
planning_request.collision_safety_margin=0.25

planning_request.start_waypoint = RR.RobotRaconteurVarValue(start_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")
planning_request.goal_waypoint = RR.RobotRaconteurVarValue(end_waypoint, "com.robotraconteur.robotics.planning.JointWaypoint")

plan_generator = c.plan(planning_request)
res = plan_generator.Next()
plan_generator.Close()

joint_trajectory=res.joint_trajectory
print(res.joint_trajectory)