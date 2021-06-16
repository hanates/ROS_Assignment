#!/bin/bash
sleep 3
rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2.5,0,0} 
rostopic pub --once /control arm_lib/control_cmd catch
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2.5,0,1.5}
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2,0,0.5}
# rostopic pub --once /control arm_lib/control_cmd release
# rostopic pub --once /ik_angles arm_lib/actuator_pos -- {2.5,0,0}