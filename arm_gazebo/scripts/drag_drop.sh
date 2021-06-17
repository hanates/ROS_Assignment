#!/bin/bash
sleep 3
#  Pose: -0.179457 -3.225999 0.78532
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {3,1,0} 
# rostopic pub --once /control arm_lib/control_cmd catch
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2.5,0,1.5}
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2,0,0.5}
# rostopic pub --once /control arm_lib/control_cmd release
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2.5,0,0}
rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2.3,0.28,0.25} 
rostopic pub --once /control arm_lib/control_cmd catch
rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2,-1.3,1.5}
rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2,-1.3,0.3}
rostopic pub --once /control arm_lib/control_cmd release
rostopic pub --once /ik_angles arm_lib/actuator_pos -- {0,0,3.4}