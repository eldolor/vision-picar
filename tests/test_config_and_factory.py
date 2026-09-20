"""
Config and backend-selection edge cases.

Small, unglamorous branches that nothing else reached: the environment
overrides the brain reads at startup (they are how the ECS deployment is
configured at all -- config/robot.yaml in the image is the same for every
task), the rejection of an unknown config key, and robot/factory.py's
refusal to pretend it can build a backend it does not have.

The factory ones matter more than their size suggests: `mode: hardware` is
the setting the whole project is aiming at, and until robot/hardware_robot.py
exists the only correct behaviour is a clear refusal rather than a confusing
ImportError on hardware day.
"""

import pytest

from control.brain_config import load_brain_config


def write(tmp_path, body="brain:\n  max_steps: 5\n"):
    p = tmp_path / "robot.yaml"
    p.write_text(body)
    return str(p)


def test_an_unknown_key_in_the_brain_block_is_refused(tmp_path):
    """A typo in config must not silently do nothing -- these keys are the
    only difference between brain-on-Pi and brain-on-Fargate."""
    path = write(tmp_path, "brain:\n  robot_urls: http://oops\n")
    with pytest.raises(ValueError) as e:
        load_brain_config(path)
    assert "robot_urls" in str(e.value)


@pytest.mark.parametrize("env,key,value,expected", [
    ("ROBOT_URL", "robot_url", "http://pi.local:8000", "http://pi.local:8000"),
    ("VISION_URL", "vision_url", "http://vision.internal", "http://vision.internal"),
    ("NAVIGATE_MODEL_ID", "navigate_model_id", "qwen.qwen3-vl-235b-a22b",
     "qwen.qwen3-vl-235b-a22b"),
    ("RECORDING_PROXY_URL", "recording_proxy_url", "http://peer", "http://peer"),
    ("RECORDING_PROXY_SECRET", "recording_proxy_secret", "s3cret", "s3cret"),
])
def test_the_environment_overrides_the_file(tmp_path, monkeypatch, env, key, value, expected):
    """How every ECS task is configured: one image, one config/robot.yaml,
    differing only by environment."""
    monkeypatch.setenv(env, value)
    assert load_brain_config(write(tmp_path))[key] == expected


# An empty value is not an override at all -- brain_config only reads the
# variable when it is set to something -- so it is not in this table.
@pytest.mark.parametrize("raw,expected", [("1", True), ("true", True), ("YES", True),
                                          ("0", False), ("false", False)])
def test_allow_recording_reads_as_a_flag_not_a_string(tmp_path, monkeypatch, raw, expected):
    """"false" is a non-empty string and would be truthy read naively -- which
    would turn recording ON for a brain configured to refuse it."""
    monkeypatch.setenv("ALLOW_RECORDING", raw)
    assert load_brain_config(write(tmp_path))["allow_recording"] is expected


# ---------- robot/factory.py ----------


def test_hardware_mode_refuses_clearly_instead_of_failing_obscurely(tmp_path):
    """Phase 11's whole promise is that the swap is a config change. Until
    robot/hardware_robot.py exists, `mode: hardware` has to say so plainly --
    an ImportError on hardware day would be a bad first impression."""
    from robot.factory import get_robot

    path = tmp_path / "robot.yaml"
    path.write_text("mode: hardware\n")
    with pytest.raises(NotImplementedError) as e:
        get_robot(str(path))
    assert "hardware" in str(e.value).lower()


def test_an_unknown_mode_names_itself(tmp_path):
    from robot.factory import get_robot

    path = tmp_path / "robot.yaml"
    path.write_text("mode: submarine\n")
    with pytest.raises(ValueError) as e:
        get_robot(str(path))
    assert "submarine" in str(e.value)


def test_sensor_noise_is_wired_in_only_when_enabled(tmp_path):
    """S5's model is opt-in: with it off, get_distance() keeps the exact
    multiples-of-30 behaviour every older test was written against."""
    from robot.factory import get_robot

    off = tmp_path / "off.yaml"
    off.write_text("mode: sim\n")
    assert get_robot(str(off)).sensor is None

    on = tmp_path / "on.yaml"
    on.write_text("mode: sim\nsim:\n  sensor_noise:\n    enabled: true\n")
    assert get_robot(str(on)).sensor is not None


# ---------- world/factory.py (N1) ----------
#
# robot/factory.py's sibling, and the same three cases matter for the same
# reasons: the default is honest, an unbuilt backend refuses by name, and
# an unknown mode says what it did not understand.


def test_world_defaults_to_no_mapper(tmp_path):
    """Nothing in this project can build a map yet, so `none` is not a
    fallback -- it is the accurate description of every deployment that
    exists. A config with no `world:` block at all must land there."""
    from world.factory import get_world
    from world.interface import NullWorld

    path = tmp_path / "robot.yaml"
    path.write_text("mode: sim\n")
    world = get_world(str(path))
    assert isinstance(world, NullWorld)
    assert world.get_pose()["usable"] is False
    assert world.get_map()["usable"] is False


def test_the_shipped_config_has_no_mapper(tmp_path):
    """And the real config/robot.yaml agrees, so a reader is never told
    one thing by the file and another by the default."""
    from world.factory import get_world
    from world.interface import NullWorld

    assert isinstance(get_world(), NullWorld)


@pytest.mark.parametrize("mode,expected", [("sim", "mock_world"), ("ros", "ros_world")])
def test_an_unbuilt_world_backend_refuses_by_name(tmp_path, mode, expected):
    """Same rule as `mode: hardware`. A silent downgrade to "no map" is
    the worst possible failure here, because it looks exactly like a
    mapper that has not converged yet -- which is the thing you would be
    debugging on the day it happened."""
    from world.factory import get_world

    path = tmp_path / "robot.yaml"
    path.write_text(f"world:\n  mode: {mode}\n")
    with pytest.raises(NotImplementedError) as e:
        get_world(str(path))
    assert expected in str(e.value)


def test_an_unknown_world_mode_names_itself(tmp_path):
    from world.factory import get_world

    path = tmp_path / "robot.yaml"
    path.write_text("world:\n  mode: atlas\n")
    with pytest.raises(ValueError) as e:
        get_world(str(path))
    assert "atlas" in str(e.value)


def test_the_environment_overrides_the_world_block(tmp_path, monkeypatch):
    """WORLD_MODE beside ROBOT_MODE, for the same deployment reason: one
    generic image, per-environment mode from the task's env vars."""
    from world.factory import get_world

    path = tmp_path / "robot.yaml"
    path.write_text("world:\n  mode: none\n")
    monkeypatch.setenv("WORLD_MODE", "atlas")
    with pytest.raises(ValueError) as e:
        get_world(str(path))
    assert "atlas" in str(e.value)
