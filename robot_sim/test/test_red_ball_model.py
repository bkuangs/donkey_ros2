import math
from pathlib import Path
import xml.etree.ElementTree as ET


MODEL_PATH = Path(__file__).parents[1] / 'models' / 'red_ball' / 'model.sdf'


def vector(element):
    return tuple(float(value) for value in element.text.split())


def test_target_motion_contract():
    model = ET.parse(MODEL_PATH).getroot().find('model')
    assert model.findtext('allow_auto_disable') == 'false'
    ball = model.find("link[@name='ball']")
    assert ball.findtext('gravity') == 'false'
    assert ball.find('collision') is None

    velocity_control = model.find(
        "plugin[@name='gz::sim::systems::VelocityControl']"
    )
    linear_velocity = vector(velocity_control.find('initial_linear'))
    angular_velocity = vector(velocity_control.find('initial_angular'))

    speed = math.hypot(linear_velocity[0], linear_velocity[1])
    assert math.isclose(speed, 0.4)
    assert math.isclose(angular_velocity[2], speed / 3.0, rel_tol=1e-9)
