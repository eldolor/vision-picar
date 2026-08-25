# vision-picar photo analysis -- AWS Lambda

Deploys the "find the bag in my photo" feature: the web twin uploads a
phone photo, this calls Claude's vision API, and returns the same
structured scene schema `brain/vision.py` uses in sim, plus a room guess.

## Why Lambda here (and only here)

This is the *analyze-a-photo* endpoint, not the robot control loop. It's
a good fit for Lambda because it needs to be reachable from your phone
anywhere (not just your home Wi-Fi), the API key can live safely in
Lambda's environment instead of on a device, and it costs basically
nothing at hobbyist volume. The actual drive/steer/stop control loop
(`robot/server.py`, Phase 9) stays local to the Pi/MacBook once hardware
exists -- safety-critical control shouldn't depend on a cloud hop being
up. See the main README for that split.

## 1. Package

```bash
cd lambda/vision_analyze
pip install -r requirements.txt -t package/
cp handler.py vision_core.py rooms_core.py package/
cd package && zip -r ../function.zip . && cd ..
```

## 2. Create the function (AWS CLI)

```bash
aws lambda create-function \
  --function-name vision-picar-analyze \
  --runtime python3.12 \
  --role <YOUR_LAMBDA_EXECUTION_ROLE_ARN> \
  --handler handler.handler \
  --zip-file fileb://function.zip \
  --timeout 15 \
  --memory-size 512
```

(15s timeout gives headroom for the vision API call, which typically
takes 1-3s -- see the cost/latency discussion earlier in this project.
512MB is comfortably enough for the anthropic SDK + JSON handling.)

## 3. Set environment variables

```bash
aws lambda update-function-configuration \
  --function-name vision-picar-analyze \
  --environment "Variables={
    ANTHROPIC_API_KEY=sk-ant-...,
    APP_SHARED_SECRET=<a long random string you generate>,
    ALLOWED_ORIGINS=https://your-deployed-web-twin.example.com
  }"
```

**Generate a shared secret** (anything long and random works):
```bash
openssl rand -hex 32
```
Store the same value in the web twin's config so it sends it as the
`x-app-secret` header on every request. Without this, a public Function
URL is an open invitation for someone else to run up your Anthropic
bill -- there's no API Gateway auth layer in front of a Function URL by
default, so this header check is the only guard.

## 4. Enable a Function URL

```bash
aws lambda create-function-url-config \
  --function-name vision-picar-analyze \
  --auth-type NONE \
  --cors '{
    "AllowOrigins": ["https://your-deployed-web-twin.example.com"],
    "AllowMethods": ["POST", "OPTIONS"],
    "AllowHeaders": ["Content-Type", "x-app-secret"]
  }'
```

`--auth-type NONE` means anyone with the URL can call it -- that's why
the shared-secret header above matters. AWS's own CORS config here is a
second layer on top of the one `handler.py` already does itself; both
need updating if you change origins.

This returns a URL like `https://<id>.lambda-url.<region>.on.aws/`.
That's what the web twin calls.

## 5. Test it (once deployed)

```bash
IMAGE_B64=$(base64 -i test_photo.jpg)
curl -X POST https://<your-function-url> \
  -H "Content-Type: application/json" \
  -H "x-app-secret: <your secret>" \
  -d "{\"image_base64\": \"$IMAGE_B64\"}"
```

## Cost guardrails worth setting

- **Reserved concurrency** — cap how many invocations can run at once
  (`aws lambda put-function-concurrency --function-name vision-picar-analyze
  --reserved-concurrent-executions 5`) so a traffic spike (malicious or
  accidental) can't run up an unbounded bill.
- **CloudWatch billing alarm** — a simple alarm on estimated charges
  catches a leaked secret or runaway retry loop before it becomes a
  surprise.
- The `APP_SHARED_SECRET` check and `ALLOWED_ORIGINS` CORS restriction
  above are the first line of defense; concurrency limits and a billing
  alarm are the backstop if those are ever bypassed.

## Keeping this in sync

`vision_core.py` and `rooms_core.py` are intentionally small, dependency-light
copies of `brain/vision.py` and `brain/rooms.py`, packaged separately so
the Lambda zip doesn't need the whole repo. If you change the prompt,
output schema, or room features in the main `brain/` modules, mirror the
change here.
