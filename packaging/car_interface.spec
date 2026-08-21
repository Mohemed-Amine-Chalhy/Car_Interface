"""PyInstaller one-folder build for the Windows desktop application."""

from __future__ import annotations

from pathlib import Path

project_root = Path(SPEC).resolve().parent.parent
source_root = project_root / "src"

datas: list[tuple[str, str]] = []
binaries: list[tuple[str, str]] = []
hidden_imports: list[str] = []

assets_directory = source_root / "car_interface" / "assets"
if assets_directory.is_dir():
    datas.append((str(assets_directory), "car_interface/assets"))

analysis = Analysis(
    [str(project_root / "main.py")],
    pathex=[str(source_root)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hidden_imports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=1,
)
python_archive = PYZ(analysis.pure)

executable = EXE(
    python_archive,
    analysis.scripts,
    [],
    exclude_binaries=True,
    name="CarInterface",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

bundle = COLLECT(
    executable,
    analysis.binaries,
    analysis.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="CarInterface",
)
