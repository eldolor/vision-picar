# Build context is the REPO ROOT -- control/brain_server.py needs the real
# brain/ and control/ packages, plus robot/interface.py + robot/safety.py
# (RobotInterface and the SafetyViolation type, which is part of that
# contract). Deliberately NOT sim/ -- the brain must never import the
# simulator or a backend directly; that's what makes "brain on the Pi" a
# deployment change instead of a code change (see PLAN-brain-relocation.md,
# and tests/test_brain_server.py's import-surface assertion).
#
#   docker build -f service/brain/Dockerfile -t vision-picar-brain .
#
# Interim cloud deployment only -- see PLAN-brain-relocation.md's "Interim:
# brain on ECS Fargate" section. Once the PiCar-X exists, the brain moves to
# the Pi and this image stops being deployed (ROBOT_URL points at localhost
# instead).

FROM python:3.12-slim

WORKDIR /app

COPY service/brain/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY brain/ /app/brain/
COPY control/ /app/control/
COPY robot/ /app/robot/
COPY config/ /app/config/

EXPOSE 8001

CMD ["uvicorn", "control.brain_server:app", "--host", "0.0.0.0", "--port", "8001"]
