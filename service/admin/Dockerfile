# Build context is the REPO ROOT -- control/admin_server.py needs
# control/brain_config.py (just to read recording_dir out of
# config/robot.yaml's brain: block) and control/admin.html. Copies the
# whole control/ package for simplicity, matching service/brain/'s and
# service/twin/'s Dockerfiles, but this process never imports
# brain_server.py, mission_runner.py, remote_robot.py, or drills.py --
# no robot/, no brain/, no anthropic, no httpx. That's the point of this
# being control/admin_server.py: viewing and deleting recorded walks has
# nothing to do with the robot or the vision service. (boto3 is in
# requirements for the scorecard's judge tier, which calls Bedrock
# directly -- a managed endpoint, not a peer service.)
#
#   docker build -f service/admin/Dockerfile -t vision-picar-admin .

FROM python:3.12-slim

WORKDIR /app

COPY service/admin/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY control/ /app/control/
COPY config/ /app/config/

EXPOSE 8002

CMD ["uvicorn", "control.admin_server:app", "--host", "0.0.0.0", "--port", "8002"]
