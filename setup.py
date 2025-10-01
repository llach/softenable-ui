# setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'softenable_display'

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
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
] + web_data_files('web')  # <— preserve tree

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=False,
    entry_points={'console_scripts': ['server = softenable_display.server_runner:main']},
)
