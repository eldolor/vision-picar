# Build context is the REPO ROOT, not this directory -- robot/server.py
# needs the real robot/, sim/, and config/ packages (not dependency-light
# copies, unlike service/vision_analyze/ -- no packaging-size constraint
# applies here, and a copy of the sim engine would drift from the real one).
#
#   docker build -f service/twin/Dockerfile -t vision-picar-twin .

FROM python:3.12-slim

WORKDIR /app

COPY service/twin/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY robot/ /app/robot/
COPY sim/ /app/sim/
COPY config/ /app/config/
COPY web-twin/ /app/web-twin/

EXPOSE 8000

CMD ["uvicorn", "robot.server:app", "--host", "0.0.0.0", "--port", "8000"]
