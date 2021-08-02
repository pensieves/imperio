import rospy
from hr_msgs.srv import SetActuatorsControl, SetActuatorsControlRequest


CONTROL_SERVICE_TOPIC = "/hr/actuators/set_control"


def set_actuator_control(
    actuator_names,
    control_type="CONTROL_ANIMATION",
    control_service=CONTROL_SERVICE_TOPIC,
):
    r"""
    CONTROL_MANUAL : control actuator by hand (code)
    CONTROL_ANIMATION : control actuator by animation
    CONTROL_DISABLE : disable actuator
    """

    service = rospy.ServiceProxy(control_service, SetActuatorsControl)
    control_request = SetActuatorsControlRequest()
    control_request.control = getattr(SetActuatorsControlRequest, control_type)
    control_request.actuators = actuator_names
    service(control_request)
