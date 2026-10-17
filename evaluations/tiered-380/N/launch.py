"""3.80.N: launch the export host (paid). Uploads the input bundle to the deploy
bucket, writes presigned links (GET for the bundle, PUT for the results), and
starts one g6.2xlarge that terminates on shutdown (DONE, error, or 3 h).

    .venv/bin/python evaluations/tiered-380/N/launch.py
"""
import io
import json
import tarfile
from pathlib import Path

import boto3

REGION, BUCKET = "us-east-2", "vision-picar-deploy-303351622021-us-east-2"
PREFIX = "tiered-380N"
AMI, TYPE = "ami-0f5f372f671b84b92", "g6.2xlarge"
HERE = Path(__file__).parent
ROOT = HERE.resolve().parents[2]
EXP = 6 * 3600

s3 = boto3.client("s3", region_name=REGION,
                  config=boto3.session.Config(signature_version="s3v4"))
put = {k: s3.generate_presigned_url("put_object", Params={"Bucket": BUCKET, "Key": f"{PREFIX}/result/{f}"},
                                    ExpiresIn=EXP)
       for k, f in {"LOG_PUT": "export.log", "NT_PUT": "onnx-NT.tar", "NVE_PUT": "onnx-NVE.tar",
                    "FORCED_PUT": "forced.json"}.items()}
env = "".join(f"{k}='{v}'\n" for k, v in put.items())
files = {"host_export.sh": HERE / "host_export.sh", "forced_internvl.py": HERE / "forced_internvl.py",
         "forced_extra.py": ROOT / "evaluations/tiered-380/K/forced_extra.py",
         "edgellm_vocab_map.py": ROOT / "tools/jetson/edgellm_vocab_map.py",
         "forced_ids.json": ROOT / "evaluations/identity-378/forced_ids.json",
         "ivl4b.jsonl": ROOT / "evaluations/tiered-380/M/ivl4b.jsonl",
         "SHA256SUMS": HERE / "host/SHA256SUMS"}
buf = io.BytesIO()
with tarfile.open(fileobj=buf, mode="w") as t:
    for name, p in files.items():
        t.add(p, arcname=name)
    data = env.encode()
    ti = tarfile.TarInfo("put.env"); ti.size = len(data)
    t.addfile(ti, io.BytesIO(data))
s3.put_object(Bucket=BUCKET, Key=f"{PREFIX}/bundle.tar", Body=buf.getvalue())
get = s3.generate_presigned_url("get_object", Params={"Bucket": BUCKET, "Key": f"{PREFIX}/bundle.tar"},
                                ExpiresIn=EXP)
user_data = f"""#!/bin/bash
shutdown -h +180
mkdir -p /opt/n && cd /opt/n
curl -sS --retry 5 -o bundle.tar '{get}' && tar -xf bundle.tar && chown -R ubuntu:ubuntu /opt/n
sudo -iu ubuntu bash /opt/n/host_export.sh
"""
ec2 = boto3.client("ec2", region_name=REGION)
r = ec2.run_instances(ImageId=AMI, InstanceType=TYPE, MinCount=1, MaxCount=1, UserData=user_data,
                      InstanceInitiatedShutdownBehavior="terminate",
                      BlockDeviceMappings=[{"DeviceName": "/dev/sda1",
                                            "Ebs": {"VolumeSize": 150, "VolumeType": "gp3",
                                                    "DeleteOnTermination": True}}],
                      TagSpecifications=[{"ResourceType": "instance",
                                          "Tags": [{"Key": "Name", "Value": "vision-picar-380N-export"}]}])
iid = r["Instances"][0]["InstanceId"]
print(json.dumps({"instance": iid, "log": f"s3://{BUCKET}/{PREFIX}/result/export.log"}))
