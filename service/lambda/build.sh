#!/usr/bin/env bash
#
# Build and upload the two Lambda zips.
#
# Run from the repo root:  bash service/lambda/build.sh <deployment-bucket>
#
# Two things here are not incidental and will bite if changed:
#
#   * --platform manylinux2014_aarch64, because the functions run on arm64.
#     pydantic and pydantic-core ship compiled wheels, so a package built on
#     a Mac installs happily and then fails at import inside Lambda with a
#     missing-.so error that says nothing about architecture. This is the
#     same trap CLAUDE.md records for ECS images, one packaging format over.
#
#   * --only-binary=:all:, so pip refuses to "helpfully" build a source
#     distribution for the local machine when a wheel is unavailable, which
#     is how the wrong architecture gets in without an error.
#
set -euo pipefail

BUCKET="${1:?usage: build.sh <deployment-bucket> [region]}"
REGION="${2:-us-east-2}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="$ROOT/.build/lambda"
STAMP="$(date -u +%Y%m%d-%H%M%S)"

rm -rf "$BUILD"; mkdir -p "$BUILD/vision" "$BUILD/walks"

deps() {  # deps <target-dir> <requirements-file>
  pip install \
    --platform manylinux2014_aarch64 \
    --implementation cp \
    --python-version 3.12 \
    --only-binary=:all: \
    --target "$1" \
    --quiet \
    -r "$2"
}

echo "==> vision"
# The service's OWN requirements, plus what Lambda adds. Derived rather than
# duplicated: a hand-written copy of this list missed pillow-heif and the
# first deploy died at import. uvicorn comes along unused, which is a few MB
# of a 250MB budget and cheaper than a list that can drift.
deps "$BUILD/vision" "$ROOT/service/vision_analyze/requirements.txt"
deps "$BUILD/vision" "$ROOT/service/lambda/requirements-vision.txt"
# FLAT, at the zip root -- app.py does `from vision_core import ...`, which
# is the layout its Dockerfile creates with one WORKDIR. Nesting these under
# service/vision_analyze/ makes that import fail at cold start. (Written
# nested first; tests/test_serverless_routes.py is what caught it.)
cp "$ROOT/service/vision_analyze/app.py"           "$BUILD/vision/"
cp "$ROOT/service/vision_analyze/vision_core.py"   "$BUILD/vision/"
cp "$ROOT/service/vision_analyze/rooms_core.py"    "$BUILD/vision/"
cp "$ROOT/service/lambda/vision_handler.py"        "$BUILD/vision/"

echo "==> walks"
deps "$BUILD/walks" "$ROOT/service/lambda/requirements-walks.txt"
mkdir -p "$BUILD/walks/control" "$BUILD/walks/config"
cp "$ROOT/control/"*.py                            "$BUILD/walks/control/"
cp "$ROOT/control/admin.html" "$ROOT/control/admin.js" "$BUILD/walks/control/" 2>/dev/null || true
cp "$ROOT/config/robot.yaml"                       "$BUILD/walks/config/"
cp "$ROOT/service/lambda/walks_handler.py"         "$BUILD/walks/"

for name in vision walks; do
  ( cd "$BUILD/$name" && zip -qr "../$name-$STAMP.zip" . -x '*.pyc' -x '*/__pycache__/*' )
  key="lambda/$name-$STAMP.zip"
  aws s3 cp "$BUILD/$name-$STAMP.zip" "s3://$BUCKET/$key" --region "$REGION" --only-show-errors
  size=$(python3 -c "import os;print(f'{os.path.getsize(\"$BUILD/$name-$STAMP.zip\")/1048576:.1f}')")
  echo "    $name: ${size}MB -> s3://$BUCKET/$key"
done

echo
echo "Deploy with:"
echo "  aws cloudformation deploy --template-file cloudformation/serverless.yaml \\"
echo "    --stack-name vision-picar-serverless --capabilities CAPABILITY_NAMED_IAM \\"
echo "    --region $REGION --parameter-overrides \\"
echo "      LambdaCodeBucket=$BUCKET \\"
echo "      VisionCodeKey=lambda/vision-$STAMP.zip \\"
echo "      WalksCodeKey=lambda/walks-$STAMP.zip \\"
echo "      VisionSharedSecret=\"\$VISION_SHARED_SECRET\" \\"
echo "      WalksSharedSecret=\"\$WALKS_SHARED_SECRET\""
echo "  (Pass both secrets: empty means the functions run with NO auth.)"
