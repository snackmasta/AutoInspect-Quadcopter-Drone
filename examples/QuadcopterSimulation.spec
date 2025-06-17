# -*- mode: python ; coding: utf-8 -*-
import os
import sys
from PyInstaller.utils.hooks import collect_data_files, collect_dynamic_libs

# Add the parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(SPECPATH), '..'))

block_cipher = None

# Collect GLFW DLL
glfw_datas = collect_data_files('glfw')
glfw_binaries = collect_dynamic_libs('glfw')

# Collect OpenGL DLLs
opengl_datas = collect_data_files('OpenGL')
opengl_binaries = collect_dynamic_libs('OpenGL')

# Collect imgui data
imgui_datas = collect_data_files('imgui')

# Additional hidden imports
hiddenimports = [
    'glfw',
    'glfw.GLFW',
    'OpenGL',
    'OpenGL.GL',
    'OpenGL.arrays',
    'OpenGL.arrays.numpymodule',
    'imgui',
    'imgui.integrations',
    'imgui.integrations.glfw',
    'numpy',
]

# Collect all data files
datas = []
datas.extend(glfw_datas)
datas.extend(opengl_datas)
datas.extend(imgui_datas)

# Collect all binary files
binaries = []
binaries.extend(glfw_binaries)
binaries.extend(opengl_binaries)

# Manually add the GLFW DLL
glfw_dll_path = os.path.join(os.path.dirname(sys.executable), 'Lib', 'site-packages', 'glfw', 'glfw3.dll')
if os.path.exists(glfw_dll_path):
    binaries.append((glfw_dll_path, '.'))

# Add quadcopter_sim package data
base_path = os.path.dirname(SPECPATH)
parent_path = os.path.dirname(base_path)
quadcopter_sim_path = os.path.join(parent_path, 'quadcopter_sim')
debug_path = os.path.join(parent_path, 'debug')

# Add debug module files as data to ensure they're available
if os.path.exists(debug_path):
    # Add the entire debug directory
    datas.append((debug_path, 'debug'))
    # Also add individual files to be sure
    for file in os.listdir(debug_path):
        if file.endswith('.py'):
            src_file = os.path.join(debug_path, file)
            datas.append((src_file, 'debug'))

a = Analysis(
    ['drone_opengl.py'],
    pathex=[
        base_path,
        parent_path,
        quadcopter_sim_path,
        debug_path,
    ],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=['pyi_rth_debug.py'],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='QuadcopterSimulation',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
