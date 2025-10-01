import os
import glob
import yaml
import requests
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from softenable_display_msgs.srv import SetDisplay

# at top of display_service_node.py
class DuplicatePresetError(Exception):
    pass


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
        import glob, yaml, os
        from collections import Counter

        if not os.path.isdir(self.configs_dir):
            self.get_logger().warn(f"No configs dir found at {self.configs_dir}")
            return

        self.presets = {}
        name_files = {}          # name -> [files it appears in] (preserve order)
        counts = Counter()       # name -> total occurrences across all YAMLs
        candidates = []          # (file, name, spec) valid entries

        for path in sorted(glob.glob(os.path.join(self.configs_dir, '**', '*.yaml'), recursive=True)):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    doc = yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().warn(f"Failed to load {path}: {e}")
                continue

            if not isinstance(doc, dict):
                self.get_logger().warn(f"Skip {path}: top-level must be a mapping")
                continue

            seen_in_file = set()
            for name, spec in doc.items():
                # track occurrences (including duplicates within the same file)
                counts[name] += 1
                name_files.setdefault(name, []).append(path)

                if not isinstance(name, str):
                    self.get_logger().warn(f"Skip entry with non-string key in {path}")
                    continue
                if not isinstance(spec, dict):
                    self.get_logger().warn(f"Skip '{name}' in {path}: value must be a mapping")
                    continue

                extra = set(spec.keys()) - {"text", "image"}
                if extra:
                    self.get_logger().warn(f"Skip '{name}' in {path}: unknown keys {sorted(extra)}")
                    continue
                if "text" not in spec or not isinstance(spec["text"], str) or not spec["text"]:
                    self.get_logger().warn(f"Skip '{name}' in {path}: 'text' is required non-empty string")
                    continue

                # normalize optional image relative to webroot/images
                image_rel = spec.get("image", "")
                if image_rel is None:
                    image_rel = ""
                if not isinstance(image_rel, str):
                    self.get_logger().warn(f"Skip '{name}' in {path}: 'image' must be string if provided")
                    continue
                if image_rel:
                    normalized = image_rel.lstrip('/').replace('\\', '/')
                    if not normalized.startswith('images/'):
                        normalized = f'images/{normalized}'
                    fs_path = os.path.join(self.web_dir, *normalized.split('/'))
                    if not os.path.isfile(fs_path):
                        self.get_logger().warn(f"Preset '{name}' in {path}: image not found: {fs_path}")
                    image_rel = normalized
                else:
                    image_rel = ""

                candidates.append((path, name, {"text": spec["text"], "image": image_rel}))

        # ---- duplicate names detection (across all files and within a single file) ----
        dup_names = sorted([n for n, c in counts.items() if c > 1])
        if dup_names:
            # Pretty list with paths relative to the package share so it prints like "configs/a.yaml"
            rel_lines = []
            # make paths relative to the package share dir (so they start with "configs/")
            share_dir = self.share_dir if hasattr(self, 'share_dir') else os.path.dirname(self.configs_dir)
            for n in dup_names:
                # unique paths, preserve order
                seen = []
                for p in name_files[n]:
                    if p not in seen:
                        seen.append(p)
                rels = [os.path.relpath(p, start=share_dir) if p.startswith(share_dir) else p for p in seen]
                rel_lines.append(f"  - {n}: " + ", ".join(rels))
            msg = "Duplicate preset names detected across config files:\n" + "\n".join(rel_lines)
            # raise with the full message; main() logs it cleanly
            raise DuplicatePresetError(msg)

        # commit candidates
        for _, name, spec in candidates:
            self.presets[name] = spec

        if not self.presets:
            self.get_logger().warn("No valid presets loaded.")
        else:
            self.get_logger().info(f"Loaded {len(self.presets)} preset(s) from YAML.")


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
