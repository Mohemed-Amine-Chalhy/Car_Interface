"""Cross-platform developer, quality, and release task runner.

The script deliberately keeps hardware access behind an explicit acknowledgement.
Run ``python scripts/dev.py --help`` for the complete command list.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
import platform
import shlex
import shutil
import subprocess  # nosec B404
import sys
import tempfile
import tomllib
from collections.abc import Sequence
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENV = ROOT / ".venv"
REPORTS_DIR = ROOT / ".reports"
DIST_DIR = ROOT / "dist"
SOURCE_TARGETS = ("src", "tests", "scripts", "main.py")
ENV_MARKER = "CAR_INTERFACE_DEV_RUNNER_ACTIVE"
HARDWARE_ACK_ENV = "CAR_INTERFACE_HARDWARE_ACKNOWLEDGED"
HARDWARE_ACK_FLAG = "--i-understand-this-controls-real-hardware"


class TaskError(RuntimeError):
    """A developer task could not be completed safely."""


def _display_command(command: Sequence[str]) -> str:
    if os.name == "nt":
        return subprocess.list2cmdline(command)
    return shlex.join(command)


def _run(
    command: Sequence[str],
    *,
    env: dict[str, str] | None = None,
    capture: bool = False,
) -> subprocess.CompletedProcess[str]:
    print(f"+ {_display_command(command)}", flush=True)
    # Command entries are produced by this task runner and shell=False is retained.
    return subprocess.run(  # nosec B603
        list(command),
        cwd=ROOT,
        env=env,
        check=True,
        text=True,
        capture_output=capture,
    )


def _capture(command: Sequence[str]) -> str:
    return _run(command, capture=True).stdout.strip()


def _python_module(module: str, *arguments: str) -> None:
    _run((sys.executable, "-m", module, *arguments))


def _project_targets() -> tuple[str, ...]:
    return tuple(target for target in SOURCE_TARGETS if (ROOT / target).exists())


def _is_project_environment() -> bool:
    try:
        return Path(sys.prefix).resolve() == VENV.resolve()
    except OSError:
        return False


def _relaunch_in_project_environment(arguments: Sequence[str]) -> int | None:
    """Relaunch once through uv when invoked with a system Python."""
    if os.environ.get(ENV_MARKER) == "1" or _is_project_environment():
        return None

    if not (ROOT / "uv.lock").is_file():
        raise TaskError("uv.lock is missing; generate it with 'uv lock' before running tasks")

    uv = shutil.which("uv")
    if uv is None:
        raise TaskError(
            "uv is required. Install it, then run scripts/bootstrap.ps1 or scripts/bootstrap.sh."
        )

    command = [uv, "run", "--locked", "--group", "dev"]
    if arguments and arguments[0] in {"build", "release-check"}:
        command.extend(("--group", "build"))
    command.extend(("python", str(Path(__file__).resolve()), *arguments))

    environment = os.environ.copy()
    environment[ENV_MARKER] = "1"
    try:
        return _run(command, env=environment).returncode
    except subprocess.CalledProcessError as error:
        return error.returncode


def _lock_check() -> None:
    uv = shutil.which("uv")
    if uv is None:
        raise TaskError("uv is not available on PATH")
    _run((uv, "lock", "--check"))


def _doctor() -> None:
    print(f"Repository: {ROOT}")
    print(f"Platform:   {platform.platform()}")
    print(f"Python:     {platform.python_version()} ({sys.executable})")

    if sys.version_info[:2] != (3, 13):
        raise TaskError("Python 3.13 is required; rerun the bootstrap script")

    for executable in ("git", "uv"):
        location = shutil.which(executable)
        if location is None:
            raise TaskError(f"Required executable is not on PATH: {executable}")
        print(f"{executable:10}{_capture((location, '--version'))}")

    required_modules = ("bandit", "mypy", "pytest", "ruff", "serial")
    missing = [name for name in required_modules if importlib.util.find_spec(name) is None]
    if missing:
        raise TaskError(f"Development environment is incomplete: {', '.join(missing)}")

    try:
        import tkinter
    except ImportError as error:
        raise TaskError(f"Tkinter is unavailable: {error}") from error
    try:
        tkinter.Tcl()
    except tkinter.TclError as error:
        raise TaskError(f"Tkinter is unavailable or unusable: {error}") from error

    _lock_check()
    print("Environment check passed.")


def _format(*, check: bool) -> None:
    arguments = ["format"]
    if check:
        arguments.append("--check")
    arguments.extend(_project_targets())
    _python_module("ruff", *arguments)


def _lint(*, fix: bool) -> None:
    arguments = ["check"]
    if fix:
        arguments.append("--fix")
    arguments.extend(_project_targets())
    _python_module("ruff", *arguments)


def _typecheck() -> None:
    _python_module("mypy")


def _pytest_arguments(*, hardware: bool, ci: bool) -> list[str]:
    if hardware:
        return [
            "-o",
            "addopts=-ra --strict-config --strict-markers --timeout=60",
            "-m",
            "hardware",
        ]

    arguments: list[str] = []
    if ci:
        REPORTS_DIR.mkdir(parents=True, exist_ok=True)
        arguments.append(f"--junitxml={REPORTS_DIR / 'pytest.xml'}")
    return arguments


def _test(*, hardware: bool, ci: bool, extra: Sequence[str] = ()) -> None:
    environment = os.environ.copy()
    if hardware:
        environment[HARDWARE_ACK_ENV] = "1"
    else:
        environment.pop(HARDWARE_ACK_ENV, None)
        REPORTS_DIR.mkdir(parents=True, exist_ok=True)

    arguments = _pytest_arguments(hardware=hardware, ci=ci)
    arguments.extend(extra)
    _run((sys.executable, "-m", "pytest", *arguments), env=environment)


def _export_runtime_requirements(destination: Path) -> None:
    uv = shutil.which("uv")
    if uv is None:
        raise TaskError("uv is not available on PATH")
    command = [uv, "export", "--locked", "--no-dev"]
    command.extend(
        (
            "--no-hashes",
            "--no-emit-project",
            "--output-file",
            str(destination),
        )
    )
    _run(command)


def _bandit() -> None:
    _python_module(
        "bandit",
        "--configfile",
        "pyproject.toml",
        "--recursive",
        "src/car_interface",
        "scripts",
        "main.py",
    )


def _audit() -> None:
    with tempfile.TemporaryDirectory(prefix="car-interface-audit-") as temporary_directory:
        requirements = Path(temporary_directory) / "requirements.txt"
        _export_runtime_requirements(requirements)
        _python_module("pip_audit", "--requirement", str(requirements))


def _security(*, skip_bandit: bool, skip_audit: bool) -> None:
    if not skip_bandit:
        _bandit()
    if not skip_audit:
        _audit()


def _checksums(directory: Path = DIST_DIR) -> Path:
    directory = directory.resolve()
    if not directory.is_dir():
        raise TaskError(f"Artifact directory does not exist: {directory}")
    artifacts = sorted(
        (
            path
            for path in directory.iterdir()
            if path.is_file()
            and path.name != "SHA256SUMS.txt"
            and (path.suffix in {".whl", ".zip"} or path.name.endswith(".tar.gz"))
        ),
        key=lambda path: path.name,
    )
    if not artifacts:
        raise TaskError(f"No release artifacts found in {directory}")
    manifest = directory / "SHA256SUMS.txt"
    lines = [f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}" for path in artifacts]
    manifest.write_text("\n".join(lines) + "\n", encoding="ascii", newline="\n")
    print(f"Checksums: {manifest}")
    return manifest


def _check(*, ci: bool, skip_audit: bool) -> None:
    _lock_check()
    _format(check=True)
    _lint(fix=False)
    _typecheck()
    _test(hardware=False, ci=ci)
    _security(skip_bandit=False, skip_audit=skip_audit)


def _safe_remove_generated_directory(path: Path) -> None:
    resolved = path.resolve()
    allowed = {(ROOT / "build").resolve(), DIST_DIR.resolve()}
    if resolved not in allowed:
        raise TaskError(f"Refusing to remove unexpected path: {resolved}")
    if resolved.exists():
        shutil.rmtree(resolved)


def _archive_desktop_bundle() -> Path:
    bundle = DIST_DIR / "CarInterface"
    if not bundle.is_dir():
        raise TaskError(f"PyInstaller bundle was not created: {bundle}")

    machine = platform.machine().lower()
    architecture = "x86_64" if machine in {"amd64", "x86_64"} else machine
    archive_base = DIST_DIR / f"CarInterface-windows-{architecture}"
    archive = Path(shutil.make_archive(str(archive_base), "zip", root_dir=bundle))
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    checksum = archive.with_suffix(f"{archive.suffix}.sha256")
    checksum.write_text(f"{digest}  {archive.name}\n", encoding="utf-8")
    print(f"Desktop archive: {archive}")
    print(f"Checksum:        {checksum}")
    return archive


def _smoke_desktop_bundle() -> None:
    executable = DIST_DIR / "CarInterface" / "CarInterface.exe"
    if not executable.is_file():
        raise TaskError(f"Desktop executable was not created: {executable}")
    _run((str(executable), "--version"))
    print("Desktop executable smoke test passed.")


def _build(*, clean: bool) -> None:
    if clean:
        _safe_remove_generated_directory(ROOT / "build")
        _safe_remove_generated_directory(DIST_DIR)

    # Hatchling is part of the locked build group, so release output never
    # resolves an unpinned backend in a temporary environment.
    _python_module("build", "--no-isolation")
    if os.name != "nt":
        print("Desktop bundle skipped: PyInstaller release bundles are built on Windows.")
        return

    _run(
        (
            sys.executable,
            "-m",
            "PyInstaller",
            "--noconfirm",
            "packaging/car_interface.spec",
        ),
    )
    _smoke_desktop_bundle()
    _archive_desktop_bundle()


def _git_output(*arguments: str) -> str:
    git = shutil.which("git")
    if git is None:
        raise TaskError("git is not available on PATH")
    return _capture((git, *arguments))


def _project_version() -> str:
    with (ROOT / "pyproject.toml").open("rb") as project_file:
        project = tomllib.load(project_file)
    return str(project["project"]["version"])


def _release_check(
    *,
    allow_dirty: bool,
    allow_untagged: bool,
    skip_audit: bool,
) -> None:
    if os.name != "nt":
        raise TaskError("Windows release bundles must be built on Windows")

    if not allow_dirty:
        status = _git_output("status", "--porcelain")
        if status:
            raise TaskError("Release requires a clean Git worktree")

    if not allow_untagged:
        try:
            tag = _git_output("describe", "--tags", "--exact-match", "HEAD")
        except subprocess.CalledProcessError as error:
            raise TaskError("Release commit must have an exact version tag") from error
        expected_tag = f"v{_project_version()}"
        if tag != expected_tag:
            raise TaskError(f"Expected release tag {expected_tag!r}, found {tag!r}")

    _check(ci=True, skip_audit=skip_audit)
    _build(clean=True)
    _checksums()


def _application_command(mode: str, namespace: argparse.Namespace) -> tuple[str, ...]:
    command = [sys.executable, "-m", "car_interface", "run", "--mode", mode]
    if namespace.config is not None:
        command.extend(("--config", str(namespace.config)))
    if getattr(namespace, "esp32_port", None):
        command.extend(("--esp32-port", namespace.esp32_port))
    if getattr(namespace, "lidar_port", None):
        command.extend(("--lidar-port", namespace.lidar_port))
    if getattr(namespace, "showcase", False):
        command.append("--showcase")
    return tuple(command)


def _run_simulation(namespace: argparse.Namespace) -> None:
    environment = os.environ.copy()
    environment.pop(HARDWARE_ACK_ENV, None)
    _run(_application_command("simulation", namespace), env=environment)


def _run_hardware(namespace: argparse.Namespace) -> None:
    environment = os.environ.copy()
    environment[HARDWARE_ACK_ENV] = "1"
    command = (*_application_command("hardware", namespace), HARDWARE_ACK_FLAG)
    _run(command, env=environment)


def _capture_showcase(namespace: argparse.Namespace) -> None:
    _run(
        (
            sys.executable,
            str(ROOT / "scripts" / "capture_showcase.py"),
            "--output-dir",
            str(namespace.output_dir),
        )
    )


def _add_config_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path, help="Path to an application TOML configuration")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("doctor", help="validate the local toolchain and lockfile")

    format_parser = subparsers.add_parser("format", help="format maintained Python code")
    format_parser.add_argument("--check", action="store_true", help="check without modifying files")

    lint_parser = subparsers.add_parser("lint", help="lint maintained Python code")
    lint_parser.add_argument("--fix", action="store_true", help="apply safe automatic fixes")

    subparsers.add_parser("typecheck", help="run strict mypy checking")

    test_parser = subparsers.add_parser("test", help="run the automated test suite")
    test_parser.add_argument(
        "--hardware",
        action="store_true",
        help="run only explicitly marked hardware tests",
    )
    test_parser.add_argument(
        "pytest_args",
        nargs=argparse.REMAINDER,
        help="additional pytest arguments after '--'",
    )

    security_parser = subparsers.add_parser(
        "security", help="run source and dependency vulnerability checks"
    )
    security_parser.add_argument("--skip-bandit", action="store_true")
    security_parser.add_argument("--skip-audit", action="store_true")

    check_parser = subparsers.add_parser("check", help="run the same quality gates as CI")
    check_parser.add_argument("--ci", action="store_true", help="write machine-readable reports")
    check_parser.add_argument(
        "--skip-audit",
        action="store_true",
        help="skip the network-backed dependency vulnerability audit",
    )

    simulation_parser = subparsers.add_parser(
        "run-sim",
        help="start the application with simulated devices",
    )
    _add_config_argument(simulation_parser)
    simulation_parser.add_argument(
        "--showcase",
        action="store_true",
        help="run the deterministic guided walkthrough",
    )

    hardware_parser = subparsers.add_parser(
        "run-hardware",
        help="start the application with physical devices",
    )
    _add_config_argument(hardware_parser)
    hardware_parser.add_argument("--esp32-port", help="explicit ESP32 serial port")
    hardware_parser.add_argument("--lidar-port", help="explicit RPLidar serial port")
    hardware_parser.add_argument(
        HARDWARE_ACK_FLAG,
        action="store_true",
        required=True,
        help="confirm that the command may move a real vehicle",
    )

    capture_parser = subparsers.add_parser(
        "capture-showcase",
        help="record the real simulator walkthrough as README media",
    )
    capture_parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "docs" / "assets" / "showcase",
        help="directory for the GIF and screenshots",
    )

    build_parser = subparsers.add_parser("build", help="build Python and Windows release artifacts")
    build_parser.add_argument(
        "--clean", action="store_true", help="remove prior build output first"
    )

    subparsers.add_parser("checksums", help="create a sorted SHA-256 release manifest")

    release_parser = subparsers.add_parser(
        "release-check",
        help="validate, build, checksum, and inventory a Windows release",
    )
    release_parser.add_argument("--allow-dirty", action="store_true")
    release_parser.add_argument("--allow-untagged", action="store_true")
    release_parser.add_argument("--skip-audit", action="store_true")

    return parser


def _extra_pytest_arguments(arguments: Sequence[str]) -> tuple[str, ...]:
    if arguments and arguments[0] == "--":
        return tuple(arguments[1:])
    return tuple(arguments)


def _dispatch(namespace: argparse.Namespace) -> None:
    match namespace.command:
        case "doctor":
            _doctor()
        case "format":
            _format(check=namespace.check)
        case "lint":
            _lint(fix=namespace.fix)
        case "typecheck":
            _typecheck()
        case "test":
            _test(
                hardware=namespace.hardware,
                ci=False,
                extra=_extra_pytest_arguments(namespace.pytest_args),
            )
        case "security":
            _security(
                skip_bandit=namespace.skip_bandit,
                skip_audit=namespace.skip_audit,
            )
        case "check":
            _check(ci=namespace.ci, skip_audit=namespace.skip_audit)
        case "run-sim":
            _run_simulation(namespace)
        case "run-hardware":
            _run_hardware(namespace)
        case "capture-showcase":
            _capture_showcase(namespace)
        case "build":
            _build(clean=namespace.clean)
        case "checksums":
            _checksums()
        case "release-check":
            _release_check(
                allow_dirty=namespace.allow_dirty,
                allow_untagged=namespace.allow_untagged,
                skip_audit=namespace.skip_audit,
            )
        case _:
            raise TaskError(f"Unknown command: {namespace.command}")


def main(arguments: Sequence[str] | None = None) -> int:
    raw_arguments = tuple(sys.argv[1:] if arguments is None else arguments)
    namespace = _parser().parse_args(raw_arguments)

    try:
        relaunched_status = _relaunch_in_project_environment(raw_arguments)
        if relaunched_status is not None:
            return relaunched_status
        _dispatch(namespace)
    except (TaskError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
