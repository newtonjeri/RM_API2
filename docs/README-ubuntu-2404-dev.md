# Setup — Ubuntu 24.04 dev host (x86_64 / amd64)

Profile `ubuntu-2404-dev`. Auto-detected; you never pass it.
Full detail in [SETUP.md](../../SETUP.md) — this is the card.

## Prerequisites

Ubuntu **24.04** (noble), `git`, sudo rights, network to github.com,
packages.ros.org and PyPI. Nothing else — the script installs ROS 2 Jazzy.

## Run it

```bash
mkdir -p ~/alix_ws/src
git clone --recurse-submodules https://github.com/newtonjeri/alix.git ~/alix_ws/src/alix
cd ~/alix_ws/src/alix

tools/setup/bootstrap.sh
```

The `src/` level matters — the workspace is the directory two above the repo,
and the script refuses to run if the layout is wrong.

## Then, in every new shell

```bash
source /opt/ros/jazzy/setup.bash
source ~/alix_ws/install/setup.bash
```

## Check it worked

```bash
tools/setup/check_environment.sh              # fast
tools/setup/check_environment.sh --with-tests # includes the offline suites
```

Expect **19 packages** built (14 alix + 5 realsense) and `0 failures`.
Two warnings are normal: `rmw_zenoh_cpp not installed` (middleware is installed
but not switched on — SETUP.md §3.2) and `branch 'develop' does not exist`.

On a **fresh** machine the offline suites should report **no failures**. If you
see exactly one and it is `test_emulator_verbatim`, that is expected on a
machine that has Newton's RM_API2 checkout — SETUP.md §6 explains why.

## Useful flags

| | |
|---|---|
| `--dry-run` | print every command, run none |
| `--check-only` | report state, change nothing |
| `--sequential` | one package at a time (deterministic, slower) |
| `--skip-build` | provision without building |

## If it fails

Re-run it — every step is idempotent and redoes only what did not take.
Interrupted build? `rm -rf build/<pkg> install/<pkg>` then rebuild that package.
Everything else: SETUP.md §7.
