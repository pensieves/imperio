from hr_msgs.srv import SetActuatorsControlRequest


def set_actuator_control(
    actuator_control_service, actuator_names, control_type="CONTROL_ANIMATION"
):
    """
    Set the actuators to desired control type.

    CONTROL_DISABLE: disable actuator
    CONTROL_MANUAL: control actuator by hand (code)
    CONTROL_ANIMATION: control actuator by animation
    """
    control_request = SetActuatorsControlRequest()
    control_request.control = getattr(SetActuatorsControlRequest, control_type)
    control_request.actuators = actuator_names
    actuator_control_service(control_request)
