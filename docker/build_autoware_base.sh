#!/usr/bin/env bash
#
# build_autoware_base.sh
# -----------------------
# Build the Autoware fork (Yaquod/autoware-yaquod) into Docker images and publish
# them to ghcr.io/yaquod, so the Jetson edge only ever *pulls* — it never builds.
#
# Run this ONCE (and again only when the Autoware fork changes) on a machine with
# lots of RAM/CPU/disk. For a Jetson (arm64) target you MUST build on an arm64
# host (e.g. an arm64 Azure VM: Dpsv5 / Epsv5 Ampere Altra). Building full
# Autoware for arm64 under x86 emulation (qemu) is impractically slow.
#
# Prereqixes: docker (with buildx), git. `docker login ghcr.io` beforehand
# (or set GHCR_USER / GHCR_TOKEN below to have the script log in).
#
# Usage:
#   ./docker/build_autoware_base.sh
#   REGISTRY=ghcr.io/yaquod ./docker/build_autoware_base.sh
#   NO_PUSH=1 ./docker/build_autoware_base.sh        # build & retag only, don't push

set -euo pipefail

# ---- Config (override via environment) -------------------------------------
FORK_URL="${FORK_URL:-https://github.com/Yaquod/autoware-yaquod.git}"
FORK_BRANCH="${FORK_BRANCH:-main}"
PLATFORM="${PLATFORM:-linux/arm64}"
REGISTRY="${REGISTRY:-ghcr.io/yaquod}"
WORKDIR="${WORKDIR:-$(pwd)/.autoware-base-build}"
NO_PUSH="${NO_PUSH:-0}"

# Target tags we want the edge/compose + Dockerfile.agent to reference.
DEVEL_TAG="${REGISTRY}/autoware:universe-devel-cuda"
RUNTIME_TAG="${REGISTRY}/autoware:universe-cuda"

echo "==> Autoware base build"
echo "    fork      : ${FORK_URL} (${FORK_BRANCH})"
echo "    platform  : ${PLATFORM}"
echo "    registry  : ${REGISTRY}"
echo "    workdir   : ${WORKDIR}"

# ---- Optional non-interactive registry login -------------------------------
if [[ -n "${GHCR_TOKEN:-}" && -n "${GHCR_USER:-}" ]]; then
  echo "==> docker login ghcr.io as ${GHCR_USER}"
  echo "${GHCR_TOKEN}" | docker login ghcr.io -u "${GHCR_USER}" --password-stdin
fi

# ---- Clone / update the fork ------------------------------------------------
if [[ -d "${WORKDIR}/.git" ]]; then
  echo "==> Updating existing clone"
  git -C "${WORKDIR}" fetch --depth 1 origin "${FORK_BRANCH}"
  git -C "${WORKDIR}" checkout "${FORK_BRANCH}"
  git -C "${WORKDIR}" reset --hard "origin/${FORK_BRANCH}"
else
  echo "==> Cloning fork"
  git clone --branch "${FORK_BRANCH}" --depth 1 "${FORK_URL}" "${WORKDIR}"
fi

# ---- Build via the fork's own tooling --------------------------------------
# The fork is a standard Autoware meta-repo; docker/build.sh imports the .repos,
# runs `docker buildx bake` with docker-bake.hcl / docker-bake-cuda.hcl and
# produces the `universe-devel(-cuda)` and `universe(-cuda)` image stages.
# Confirm the exact flags on your checkout with: ./docker/build.sh --help
#
# NOTE: build.sh only *loads* the target it builds. The default target is the
# runtime `universe`; `--devel-only` loads `universe-devel`. We need BOTH:
#   - universe-devel-cuda : builder base for the agent (has colcon + headers)
#   - universe-cuda       : runtime base for the agent + the `autoware` service
# So we invoke build.sh twice (the second run is mostly cache).
echo "==> Building the Autoware runtime image (this takes a long time)"
pushd "${WORKDIR}" >/dev/null
./docker/build.sh --platform "${PLATFORM}"
echo "==> Building the Autoware devel image (mostly cached)"
./docker/build.sh --platform "${PLATFORM}" --devel-only
popd >/dev/null

# ---- Discover what build.sh produced and retag to our registry -------------
# build.sh tags images under its own repo prefix (ghcr.io/autowarefoundation/autoware
# or the fork's configured prefix). Find the freshly built universe(-devel)-cuda
# images and retag them to ${REGISTRY}.
retag() {
  local want_suffix="$1" dest="$2"
  local src
  src="$(docker images --format '{{.Repository}}:{{.Tag}}' \
        | grep -E "autoware:${want_suffix}\$" | head -n1 || true)"
  if [[ -z "${src}" ]]; then
    echo "ERROR: could not find a built image matching autoware:${want_suffix}" >&2
    echo "       Available autoware images:" >&2
    docker images | grep -i autoware >&2 || true
    exit 1
  fi
  echo "==> Retagging ${src} -> ${dest}"
  docker tag "${src}" "${dest}"
}

retag "universe-devel-cuda" "${DEVEL_TAG}"
retag "universe-cuda"       "${RUNTIME_TAG}"

# ---- Push -------------------------------------------------------------------
if [[ "${NO_PUSH}" == "1" ]]; then
  echo "==> NO_PUSH=1 set, skipping push. Local tags ready:"
  echo "    ${DEVEL_TAG}"
  echo "    ${RUNTIME_TAG}"
else
  echo "==> Pushing to ${REGISTRY}"
  docker push "${DEVEL_TAG}"
  docker push "${RUNTIME_TAG}"
fi

echo "==> Done."
echo "    devel  : ${DEVEL_TAG}"
echo "    runtime: ${RUNTIME_TAG}"
echo "Next: build & push the agent image with docker/Dockerfile.agent, then deploy"
echo "with docker/docker-compose.edge.yaml on the Jetson."
