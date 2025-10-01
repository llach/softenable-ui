import os
import glob
import yaml
import requests
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from softenable_display_msgs.srv import SetDisplay

ALLOWED_KEYS = {"text", "image"}

class DisplayService(Node):
    def __init__(self):
        super().__init__('softenable_display_service')

        # Params
        self.declare_parameter('server_host', 'localhost')
        self.declare_parameter('server_port', 8080)
        self.declare_parameter('endpoint', '/update')  # POST target

        host = self.get_parameter('server_host').get_parameter_value().string_value
        port = int(self.get_parameter('server_port').get_parameter_value().integer_value)
        endpoint = self.get_parameter('endpoint').get_parameter_value().string_value
        self.url = f'http://{host}:{port}{endpoint}'

        # Resolve package share dirs
        share_dir = get_package_share_directory('softenable_display')
        self.web_dir = os.path.join(share_dir, 'web')
        self.images_dir = os.path.join(self.web_dir, 'images')
        self.configs_dir = os.path.join(share_dir, 'configs')

        # Load & validate presets
        self.presets = {}
        self._load_presets()

        # Service
        self.srv = self.create_service(SetDisplay, 'set_display', self.handle_set_display)
        self.get_logger().info(f"Ready. POST target: {self.url}. Presets: {sorted(self.presets.keys())}")

    # --------- YAML loading / validation ----------
    def _load_presets(self):
        if not os.path.isdir(self.configs_dir):
            self.get_logger().warn(f"No configs dir found at {self.configs_dir}")
            return

        loaded = 0
        for path in sorted(glob.glob(os.path.join(self.configs_dir, '**', '*.yaml'), recursive=True)):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    doc = yaml.safe_load(f) or {}
                if not isinstance(doc, dict):
                    self.get_logger().warn(f"Skip {path}: top-level must be a mapping")
                    continue
                for name, spec in doc.items():
                    if not isinstance(name, str) or not isinstance(spec, dict):
                        self.get_logger().warn(f"Skip {path}: invalid entry for '{name}'")
                        continue
                    extra = set(spec.keys()) - ALLOWED_KEYS
                    if extra:
                        self.get_logger().warn(f"Skip '{name}' in {path}: unknown keys {sorted(extra)}")
                        continue
                    if 'text' not in spec or not isinstance(spec['text'], str) or not spec['text']:
                        self.get_logger().warn(f"Skip '{name}' in {path}: 'text' is required non-empty string")
                        continue
                    image_rel = spec.get('image', '')
                    if image_rel is None:
                        image_rel = ''
                    if not isinstance(image_rel, str):
                        self.get_logger().warn(f"Skip '{name}' in {path}: 'image' must be string")
                        continue

                    # Normalize image path (relative to webroot/images), and validate if present
                    if image_rel:
                        # allow both "iri_logo.png" and "images/iri_logo.png"
                        normalized = image_rel.lstrip('/').replace('\\', '/')
                        if not normalized.startswith('images/'):
                            normalized = f'images/{normalized}'
                        fs_path = os.path.join(self.web_dir, *normalized.split('/'))
                        if not os.path.isfile(fs_path):
                            self.get_logger().warn(f"Preset '{name}': image file not found: {fs_path}")
                            # allow anyway, frontend will just hide if 404? better to keep strict:
                            # continue
                        image_rel = normalized
                    else:
                        image_rel = ''

                    self.presets[name] = {
                        'text': spec['text'],
                        'image': image_rel
                    }
                    loaded += 1
            except Exception as e:
                self.get_logger().warn(f"Failed to load {path}: {e}")

        if loaded == 0:
            self.get_logger().warn("No valid presets loaded.")
        else:
            self.get_logger().info(f"Loaded {loaded} preset(s) from YAML.")

    # --------- Service handler ----------
    def handle_set_display(self, req: SetDisplay.Request, res: SetDisplay.Response):
        name = req.name
        preset = self.presets.get(name)
        if not preset:
            self.get_logger().warn(f"Preset '{name}' not found.")
            res.success = False
            return res

        payload = {
            'text': preset['text'],
            # send empty string to clear image, or 'images/...' to show
            'image': preset['image']
        }

        try:
            r = requests.post(self.url, json=payload, timeout=2.0)
            if r.ok:
                self.get_logger().info(f"Applied preset '{name}'")
                res.success = True
            else:
                self.get_logger().error(f"Server returned {r.status_code}: {r.text}")
                res.success = False
        except Exception as e:
            self.get_logger().error(f"POST {self.url} failed: {e}")
            res.success = False
        return res

def main():
    rclpy.init()
    node = DisplayService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
