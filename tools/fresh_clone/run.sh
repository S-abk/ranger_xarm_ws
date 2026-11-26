#!/bin/bash
# Prove that a commit of this repository goes from `git clone` to a built,
# smoke-tested simulation on a machine with nothing but Ubuntu 24.04 and
# ROS 2 Jazzy, by following the README's own code blocks in a container.
#
#   tools/fresh_clone/run.sh [--ref COMMIT] [--drivers] [--out DIR]
#
#   --ref      commit to test (default: HEAD of this checkout; committed
#              changes only)
#   --drivers  also run the README's physical-robot dependency blocks (build
#              only: nothing is launched) and the missing-package check
#   --out      log directory (default: a new temporary directory); the
#              commit's throwaway clone is made beside it, as DIR.src
#
# Needs: Docker usable without sudo, network access to GitHub, the ROS and
# Ubuntu package mirrors, about 10 GB of free disk and 8 GB of memory. Takes
# 20-30 minutes, most of it apt. On the host this script only clones the
# commit and starts the container; everything else (tools/fresh_clone/
# walkthrough.sh) runs inside it, as the container's root, with no access to
# host devices and the default bridge network. The result is
# $OUT/steps.tsv, one line per step, each marked OK or UNEXPECTED.
set -euo pipefail
HERE=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)
WS=$(cd "$HERE/../.." && pwd)
REF=HEAD DRIVERS=0 OUT=
while [ $# -gt 0 ]; do
  case $1 in
    --ref) REF=$2; shift 2;;
    --drivers) DRIVERS=1; shift;;
    --out) OUT=$2; shift 2;;
    *) sed -n '2,20p' "$0"; exit 2;;
  esac
done
IMAGE=ubuntu:24.04@sha256:562111b88caa79c71391c0c4d3c2114a7d5c35915c20561fd893cb1e07929e6b
OUT=${OUT:-$(mktemp -d -t ranger_xarm_fresh_clone.XXXX)}
mkdir -p "$OUT"
SHA=$(git -C "$WS" rev-parse --verify "$REF^{commit}")

# A throwaway clone whose HEAD is the commit under test, so the README's
# plain `git clone` gets exactly that commit. It sits beside the log
# directory, so the container mounts nothing outside the two.
OUT=$(cd "$OUT" && pwd)
SRC=$OUT.src
rm -rf "$SRC"
trap 'rm -rf "$SRC"' EXIT
git clone -q --no-hardlinks "$WS" "$SRC"
git -C "$SRC" checkout -q --detach "$SHA"
cp "$HERE/walkthrough.sh" "$SRC.walkthrough.sh"
trap 'rm -rf "$SRC" "$SRC.walkthrough.sh"' EXIT

# Mounts: the clone and the script read-only, the log directory read-write.
# No other host path, no devices, no privileges, the default bridge network.
NAME=ranger_xarm_fresh_clone_$$
echo "testing $SHA in container $NAME; logs in $OUT"
rc=0
docker run --rm --name "$NAME" \
  --memory=8g --memory-swap=8g --shm-size=1g --cpus=8 \
  -e FRESH_CLONE_CONTAINER=1 -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -e REPO=/repo -e REF="$SHA" -e DRIVERS="$DRIVERS" -e OUT=/out \
  -v "$SRC:/repo:ro" -v "$SRC.walkthrough.sh:/walkthrough.sh:ro" -v "$OUT:/out:rw" \
  "$IMAGE" bash /walkthrough.sh > "$OUT/docker.log" 2>&1 || rc=$?
column -t -s $'\t' "$OUT/steps.tsv" 2>/dev/null || cat "$OUT/steps.tsv"
if [ $rc -ne 0 ] || grep -q UNEXPECTED "$OUT/steps.tsv" || ! grep -q '^done' "$OUT/walkthrough.log"; then
  echo "fresh-clone proof FAILED (logs: $OUT)"; exit 1
fi
echo "fresh-clone proof PASSED (logs: $OUT)"
