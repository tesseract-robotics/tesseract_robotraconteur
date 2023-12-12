from RobotRaconteur.Client import *
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.UuidUtil import UuidUtil
import numpy as np

def main():
    
    c = RRN.ConnectService('rr+tcp://localhost:63158?service=tesseract')

    geom = GeometryUtil(client_obj=c)
    uuid_util = UuidUtil(client_obj=c)

    command_lang_const = RRN.GetConstants("experimental.tesseract_robotics.command_language",c)
    action_const = RRN.GetConstants("com.robotraconteur.action",c)

    cart_wp_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.CartesianWaypoint",c)
    move_instr_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.MoveInstruction",c)
    composite_instr_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.CompositeInstruction",c)
    manip_info_type = RRN.GetStructureType("experimental.tesseract_robotics.common.ManipulatorInfo",c)
    planning_problem_type = RRN.GetStructureType("experimental.tesseract_robotics.tasks.planning.PlanningTaskComposerProblem",c)
    task_exec_input_type = RRN.GetStructureType("experimental.tesseract_robotics.tasks.TaskExecutorInput",c)

    # Get tasks
    task_info = c.task_pipelines_info
    output_key = task_info["FreespacePipeline"].output_keys[0]

    # Create the environment
    c.load_environment("default", "env")

    def build_move_instr(xyz, rpy):
        wp = cart_wp_type()
        wp.transform = geom.xyz_rpy_to_transform(xyz,rpy)

        instr = move_instr_type()
        instr.waypoint = RR.VarValue(wp, "experimental.tesseract_robotics.command_language.CartesianWaypoint")
        instr.profile = "DEFAULT"
        instr.move_type = command_lang_const["MoveInstructionType"]["freespace"]
        instr.uuid = uuid_util.NewRandomUuid()
        return RR.VarValue(instr, "experimental.tesseract_robotics.command_language.MoveInstruction")


    instr1 = build_move_instr([0.8, -0.3, 1.455], np.deg2rad([0,90,0]))
    instr2 = build_move_instr([0.8, 0.3, 1.455], np.deg2rad([0,90,0]))
    instr3 = build_move_instr([0.8, 0.5, 1.455], np.deg2rad([0,90,0]))

    manip_info = manip_info_type()
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    manip_info.manipulator = "manipulator"

    composite = composite_instr_type()
    composite.order = command_lang_const["CompositeInstructionOrder"]["ordered"]
    composite.instructions = [instr1, instr2] #, instr3]
    composite.manipulator_info = manip_info
    composite.uuid = uuid_util.NewRandomUuid()

    problem = planning_problem_type()
    problem.name = "example"
    problem.input = RR.VarValue(composite, "experimental.tesseract_robotics.command_language.CompositeInstruction")
    problem.environment_name = "env"
    problem.manip_info = manip_info

    executor = c.get_task_executors("default")

    exec_input = task_exec_input_type()
    exec_input.pipeline_name = "FreespacePipeline"
    exec_input.problem = RR.VarValue(problem, "experimental.tesseract_robotics.tasks.planning.PlanningTaskComposerProblem")

    exec_gen = executor.run(exec_input)

    status = None
    while True:
        res, status1 = exec_gen.TryNext()
        if not res:
            break
        status = status1
    
    # print(status)

    assert status.action_status == action_const["ActionStatusCode"]["complete"]

    output_plan = status.data_storage[output_key]

    wp_positions = []

    for instr_v in output_plan.data.instructions:
        instr = instr_v.data
        wp = instr.waypoint.data
        wp_positions.append(wp.position)

    print(wp_positions)
        






if __name__ == '__main__':
    main()