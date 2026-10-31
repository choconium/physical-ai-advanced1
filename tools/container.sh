#!/usr/bin/env bash
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
container=physical-ai-advanced1-sim
image=physical-ai-advanced1:2.3.1
case "${1:-}" in
  build)
    export BUILDX_CONFIG="$project/artifacts/docker-buildx"
    mkdir -p "$BUILDX_CONFIG"
    docker build -t "$image" -f "$project/docker/Dockerfile" "$project/docker"
    ;;
  start)
    # Private bridge; no host network, hardware devices or published ports.
    docker run -d --name "$container" --init --shm-size=2g \
      --env ROS_DOMAIN_ID=64 --mount "type=bind,src=$project,dst=/project" "$image"
    ;;
  exec) shift; docker exec -w /project "$container" bash /project/tools/ros_env.sh "$@" ;;
  stop) docker stop "$container" ;;
  remove) docker rm "$container" ;;
  *) echo 'usage: container.sh {build|start|exec COMMAND...|stop|remove}' >&2; exit 2 ;;
esac
