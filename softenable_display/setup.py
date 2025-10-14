# setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'softenable_display'

def dir_data_files(src_root, dest_root):
    entries = []
    for dirpath, _, filenames in os.walk(src_root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, src_root)
        dest = os.path.join('share', package_name, dest_root) if rel == '.' \
               else os.path.join('share', package_name, dest_root, rel)
        sources = [os.path.join(dirpath, f) for f in filenames]
        entries.append((dest, sources))
    return entries

def web_data_files(src_root='web'):
    entries = []
    for dirpath, _, filenames in os.walk(src_root):
        if not filenames:
            continue
        rel = os.path.relpath(dirpath, src_root)  # ".", "images", etc.
        dest = os.path.join('share', package_name, 'web') if rel == '.' \
               else os.path.join('share', package_name, 'web', rel)
        sources = [os.path.join(dirpath, f) for f in filenames]
        entries.append((dest, sources))
    return entries

data_files = [
    (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
     [f'resource/{package_name}']),
    (os.path.join('share', package_name), ['package.xml']),
    (os.path.join('share', package_name), glob('resource/*.ui')),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
] + web_data_files('web')  + dir_data_files('configs', 'configs')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'ament_index_python', 'PyYAML', 'requests'],
    zip_safe=False,
    entry_points={'console_scripts': [
        'gui = softenable_display.gui:main',
        'server = softenable_display.server_runner:main',
        'tts_service = softenable_display.tts_service:main',
        'change_display = softenable_display.change_display:main',
        'display_service = softenable_display.display_service_node:main',
    ]},
)
