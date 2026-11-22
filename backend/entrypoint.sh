#!/bin/sh
set -eu
mkdir -p /data/configuration /data/configurations
for f in drover_config.json; do
  [ -f "/data/configuration/$f" ] || cp "/app/configuration/$f" "/data/configuration/$f"
done
for f in physics_config.json environment_config.json; do
  [ -f "/data/configurations/$f" ] || cp "/app/configurations/$f" "/data/configurations/$f"
done
export DROVER_CONFIG_DIR=/data
export DROVER_DB_PATH=${DROVER_DB_PATH:-/data/drover.sqlite3}
exec uvicorn backend.api.main:app --host 0.0.0.0 --port "${PORT:-8000}"
