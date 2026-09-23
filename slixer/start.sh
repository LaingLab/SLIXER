#!/usr/bin/env bash
# Starts Slixer from anywhere: ./slixer/start.sh, or add --camera, --host 0.0.0.0, and so on.
# Everything after the script's name is passed straight to run.py. The first run sets up the environment.
set -e
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
exec uv run --project "$here/.." "$here/run.py" "$@"
