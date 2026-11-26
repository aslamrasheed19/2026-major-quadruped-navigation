"""Every key in config/walk.yaml and config/nav.yaml must be a parameter the node declares (ROS silently ignores unknown keys),
and the final end-to-end run is driven by the YAML values exactly as the launch file would pass them."""
import os, sys, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_node_mj as T
from fakeros import BUS
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ov = {}
for f in ("walk.yaml", "nav.yaml"):
    ov.update(yaml.safe_load(open(os.path.join(PKG, "config", f)))["svan_walk_controller"]["ros__parameters"])
mode = sys.argv[1] if len(sys.argv) > 1 else "sensed"
ov.update(nav_mode=mode, payload_kg=0.83 if mode == "sensed" else 0.0, zero_offset=False, mode="path")
c, b = ov["obstacle_circles"], ov["obstacle_boxes"]
circles = [tuple(c[i:i + 3]) for i in range(0, len(c), 3)]; boxes = [tuple(b[i:i + 4]) for i in range(0, len(b), 4)]
res, node = T.run(mode, W=[tuple(ov["waypoints"][i:i + 2]) for i in range(0, len(ov["waypoints"]), 2)], circles=circles, boxes=boxes, extra=ov, quiet=True)
unknown = [k for k in ov if k not in node._params]
print("YAML keys not declared by the node:", unknown or "none")
print("%s: done=%s t=%.1f end=(%.2f,%.2f) min_gap=%.3f replans=%d" % (mode, res["done"], res["t"], *res["end"], res["min_clear"], res["replans"]))
