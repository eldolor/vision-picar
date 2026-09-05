#!/usr/bin/env bash
#
# Upload the SPA and the admin console to the static bucket, then invalidate
# CloudFront.
#
# Run from the repo root:
#   bash service/static/sync.sh <static-bucket> <distribution-id> [region]
#
# Both values come from the serverless stack's outputs:
#   aws cloudformation describe-stacks --stack-name vision-picar-serverless \
#     --query 'Stacks[0].Outputs' --output table
#
# `aws s3 sync` is deliberately NOT used. It guesses content types from file
# extensions, and one of these files has none on purpose: control/admin.html
# is served at /admin, so its key is extensionless and a guessed type makes
# the browser download the console instead of rendering it. Each object is
# put individually with the type and cache policy from assets.json.
#
set -euo pipefail

BUCKET="${1:?usage: sync.sh <static-bucket> <distribution-id> [region]}"
DIST="${2:?usage: sync.sh <static-bucket> <distribution-id> [region]}"
REGION="${3:-us-east-2}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MANIFEST="$ROOT/service/static/assets.json"

# Read the manifest once; the test reads the same file, so the two cannot
# drift into disagreeing about what the site consists of.
#
# A `while read` loop rather than `mapfile`: macOS ships bash 3.2, which has
# no mapfile, and the failure is silent -- the array comes back empty, the
# loop body never runs, and the script uploads nothing and exits 0. That is
# the same "looks fine, does nothing" shape as the /app.js bug this project
# has now shipped five times, so it is worth the two extra lines.
rows() {
  python3 -c "
import json
for a in json.load(open('$MANIFEST'))['assets']:
    print('\t'.join([a['src'], a['key'], a['type'], a['cache']]))
"
}

COUNT=$(rows | wc -l | tr -d ' ')
[ "$COUNT" -gt 0 ] || { echo "assets.json produced no rows -- refusing to sync nothing"; exit 1; }
echo "==> uploading $COUNT objects to s3://$BUCKET"

uploaded=0
while IFS=$'\t' read -r src key ctype cache; do
  [ -n "$src" ] || continue
  [ -f "$ROOT/$src" ] || { echo "MISSING: $src (listed in assets.json)"; exit 1; }
  case "$cache" in
    none) cc="no-cache, must-revalidate" ;;
    day)  cc="public, max-age=86400" ;;
    *)    echo "unknown cache policy: $cache"; exit 1 ;;
  esac
  aws s3api put-object \
    --bucket "$BUCKET" --key "$key" \
    --body "$ROOT/$src" \
    --content-type "$ctype" \
    --cache-control "$cc" \
    --region "$REGION" \
    --output text --query 'ETag' >/dev/null
  printf '    %-34s -> %s\n' "$src" "$key"
  uploaded=$((uploaded + 1))
done < <(rows)

# Belt to the braces above: if the loop somehow ran zero times we would
# otherwise invalidate a distribution over an unchanged bucket and report
# success.
[ "$uploaded" -eq "$COUNT" ] || { echo "uploaded $uploaded of $COUNT -- aborting"; exit 1; }

# Only the no-cache objects actually need this -- the icons are content-stable
# and carry a day of TTL on purpose -- but a wildcard invalidation is one
# request either way and removes the chance of forgetting one. CloudFront
# gives 1000 free invalidation paths a month; this uses one.
echo "==> invalidating $DIST"
aws cloudfront create-invalidation --distribution-id "$DIST" --paths '/*' \
  --query 'Invalidation.[Id,Status]' --output text
