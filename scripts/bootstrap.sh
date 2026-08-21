#!/usr/bin/env sh
set -eu

usage() {
    printf '%s\n' "Usage: scripts/bootstrap.sh [--check]"
}

run_checks=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --check) run_checks=1 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
    shift
done

repository_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
cd "$repository_root"

if ! command -v uv >/dev/null 2>&1; then
    printf '%s\n' "uv is required. Install it from https://docs.astral.sh/uv/getting-started/installation/." >&2
    exit 1
fi

python_version=$(tr -d '[:space:]' < .python-version)
uv python install "$python_version"

uv sync --locked --all-groups

uv run --locked --group dev pre-commit install --install-hooks --hook-type pre-commit --hook-type pre-push
uv run --locked --group dev python scripts/dev.py doctor

if [ "$run_checks" -eq 1 ]; then
    uv run --locked --group dev python scripts/dev.py check
fi

printf '%s\n' "Environment ready. Run: uv run python scripts/dev.py run-sim"
