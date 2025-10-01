import os
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory

def main():
    share_dir = get_package_share_directory('softenable_display')
    web_dir = os.path.join(share_dir, 'web')
    server_js = os.path.join(web_dir, 'server.js')

    if not os.path.exists(server_js):
        print(f"[softenable_display] server.js not found at {server_js}", file=sys.stderr)
        return 1

    env = os.environ.copy()
    env['WEB_ROOT'] = web_dir   # server.js uses this as its static root
    # Optional: respect PORT env if user sets it; server.js already reads it.

    try:
        # run "node server.js" with cwd at web_dir
        proc = subprocess.Popen(
            ['node', server_js],
            cwd=web_dir,
            env=env
        )
        proc.wait()
        return proc.returncode
    except FileNotFoundError:
        print("[softenable_display] 'node' not found. Please install Node.js and ensure it is in PATH.", file=sys.stderr)
        return 127
