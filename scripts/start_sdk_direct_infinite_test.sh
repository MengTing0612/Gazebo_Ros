#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "scripts/start_sdk_direct_infinite_test.sh is a compatibility entry."
echo "Current default evaluation uses residual RC control; forwarding to:"
echo "  scripts/start_residual_infinite_test.sh"
echo

exec "$ROOT_DIR/scripts/start_residual_infinite_test.sh" "$@"
