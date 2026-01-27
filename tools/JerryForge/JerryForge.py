#!/usr/bin/env python3
import json, secrets, string, os, math, sys, glob, argparse, re
from typing import List, Dict, Optional, Tuple, Any

# ==========================================
# CONFIGURATION
# ==========================================
DEFAULT_CONFIG_FILE = "jerry_plotter-defaults.json"
COORD_TOLERANCE = 0.5  # matching tolerance for reverse casting (units)

def load_config(file_path: str = DEFAULT_CONFIG_FILE) -> Dict[str, Any]:
    fallback: Dict[str, Any] = {
        # Thinning (JerryIO output)
        "density": 1,
        "xy_tol": 2.0,
        "theta_tol": 5.0,

        # Visuals (JerryIO visibility)
        "visible_xy_tol": 15.0,
        "visible_theta_tol": 35.0,
        "vis_interval": 1000,

        # Ease of use
        "offset_x": 0.0,
        "offset_y": 0.0,
        "offset_theta": 0.0,

        # Quality of life and visual settings
        "robot_width": 12,
        "robot_height": 12,
        "jerryio_point_density": 2,

        # File searching locations
        "scan_paths_latest": ["./"],
        "scan_paths_batch": ["./"],
        "output_folder": "",

        # PathCaster
        "render_invisible_waypoints": False,
        "template": "Fallback template. I = ${iteration}.",

        # Viewer output toggle + viewer thinning
        "export_viewer_json": False,
        "viewer_thin_ms": 20,     # min time delta between kept points (if timestamps exist)
        "log_hz": 100,            # used only if timestamps are missing
    }

    if os.path.exists(file_path):
        try:
            with open(file_path) as f:
                cfg = json.load(f)
            for k, v in fallback.items():
                cfg.setdefault(k, v)
            return cfg
        except Exception as e:
            print(f"Warning: Could not read {file_path}, using fallback. {e}")
            return fallback

    print(f"File: {file_path} could not be read, using fallback")
    return fallback

# ==========================================
# UTILS
# ==========================================
def get_logs_from_paths(paths: List[str]) -> List[str]:
    logs: List[str] = []
    for p in paths:
        logs.extend(glob.glob(os.path.join(os.path.expanduser(p), "*.log")))
    return logs

def sort_logs(logs: List[str], newest: bool = True) -> List[str]:
    return sorted(logs, key=os.path.getmtime, reverse=newest)

def generate_uid() -> str:
    return "".join(secrets.choice(string.ascii_letters + string.digits) for _ in range(10))

def normalize_heading(theta: float) -> float:
    return round((theta % 360 + 360) % 360, 3)

def get_angular_diff(a: float, b: float) -> float:
    diff = abs(a - b) % 360
    return 360 - diff if diff > 180 else diff

def confirm(msg: str) -> bool:
    return input(f"{msg} (y/n): ").strip().lower() == "y"

def sanitize_batch_name(base: Optional[str], original: str) -> str:
    return f"{base}_{original}" if base else original

def ensure_output_folder(path: str) -> str:
    if not path or re.search(r"[*:?<>|]", path):
        return "./"
    os.makedirs(path, exist_ok=True)
    return path

def _prompt_bool(prompt: str, default: bool) -> bool:
    d = "y" if default else "n"
    v = input(f"{prompt} (y/n) [{d}]: ").strip().lower()
    if not v:
        return default
    return v.startswith("y")

def prompt_for_config(defaults: Dict[str, Any]) -> Dict[str, Any]:
    """
    Interactive config builder. Keeps parity with load_config() fallback keys.
    """
    print("\nConfigure Path Parameters (Enter = default)")

    # NOTE: keep types consistent with defaults (floats stay float)
    cfg: Dict[str, Any] = {
        # Thinning
        "density": int(input(f"Path Density [{defaults['density']}]: ") or defaults["density"]),
        "xy_tol": float(input(f"XY Tolerance [{defaults['xy_tol']}]: ") or defaults["xy_tol"]),
        "theta_tol": float(input(f"Theta Tolerance [{defaults['theta_tol']}]: ") or defaults["theta_tol"]),

        # Visibility tuning
        "visible_xy_tol": float(input(f"Visibility XY Tolerance [{defaults['visible_xy_tol']}]: ") or defaults["visible_xy_tol"]),
        "visible_theta_tol": float(input(f"Visibility Theta Tolerance [{defaults['visible_theta_tol']}]: ") or defaults["visible_theta_tol"]),
        "vis_interval": int(input(f"Visibility Interval [{defaults['vis_interval']}]: ") or defaults["vis_interval"]),

        # Offsets
        "offset_x": float(input(f"Offset X [{defaults['offset_x']}]: ") or defaults["offset_x"]),
        "offset_y": float(input(f"Offset Y [{defaults['offset_y']}]: ") or defaults["offset_y"]),
        "offset_theta": float(input(f"Offset Theta [{defaults['offset_theta']}]: ") or defaults["offset_theta"]),

        # Robot/GC config
        "robot_width": float(input(f"Robot Width [{defaults['robot_width']}]: ") or defaults["robot_width"]),
        "robot_height": float(input(f"Robot Height [{defaults['robot_height']}]: ") or defaults["robot_height"]),
        "jerryio_point_density": int(input(f"JerryIO Point Density [{defaults['jerryio_point_density']}]: ") or defaults["jerryio_point_density"]),

        # Viewer output toggle
        "export_viewer_json": _prompt_bool("Export viewer JSON instead of JerryIO", bool(defaults.get("export_viewer_json", False))),
        "viewer_thin_ms": int(input(f"Viewer thin interval (ms) [{defaults.get('viewer_thin_ms', 20)}]: ") or defaults.get("viewer_thin_ms", 20)),
        "log_hz": int(input(f"Log Hz (if no timestamps) [{defaults.get('log_hz', 100)}]: ") or defaults.get("log_hz", 100)),

        # File searching + output
        "scan_paths_latest": defaults["scan_paths_latest"],
        "scan_paths_batch": defaults["scan_paths_batch"],
        "output_folder": defaults["output_folder"],

        # Reverse casting / template
        "render_invisible_waypoints": _prompt_bool("Render invisible waypoints on cast", bool(defaults.get("render_invisible_waypoints", False))),
        "template": defaults.get("template", "Fallback template. I = ${iteration}."),
    }
    return cfg

# ==========================================
# LOG PROCESSING
# ==========================================
def _try_int(s: str) -> Optional[int]:
    try:
        if re.fullmatch(r"-?\d+", s.strip()):
            return int(s.strip())
    except Exception:
        pass
    return None

def process_log_data(lines: List[str], params: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Supports two DATA formats:
      A) Old: [DATA],x,y,theta,l_vel,r_vel
      B) New: [DATA],timestamp_ms,x,y,theta,l_vel,r_vel
    Also tolerates extra fields after r_vel (ignored).
    """
    points: List[Dict[str, Any]] = []
    synthesized_t = 0
    dt_ms = int(round(1000 / max(1, int(params.get("log_hz", 100)))))

    for line in lines:
        if "[DATA]" in line.upper():
            try:
                raw = line.split("[DATA]", 1)[1].strip()
                parts = [p.strip() for p in raw.split(",") if p.strip()]

                # Need at least x,y,theta,l,r => 5; with timestamp => 6
                if len(parts) < 5:
                    continue

                t_ms: Optional[int] = None
                maybe_ts = _try_int(parts[0])
                if maybe_ts is not None and len(parts) >= 6:
                    # Timestamped format
                    t_ms = maybe_ts
                    x_i, y_i, th_i = 1, 2, 3
                    l_i, r_i = 4, 5
                else:
                    # Old format (no timestamp)
                    t_ms = synthesized_t
                    synthesized_t += dt_ms
                    x_i, y_i, th_i = 0, 1, 2
                    l_i, r_i = 3, 4

                l_vel = float(parts[l_i])
                r_vel = float(parts[r_i])

                x = round(float(parts[x_i]) + params["offset_x"], 3)
                y = round(float(parts[y_i]) + params["offset_y"], 3)
                theta = normalize_heading(float(parts[th_i]) + params["offset_theta"])
                speed = (abs(l_vel) + abs(r_vel)) / 2.0

                points.append(
                    {
                        "t": int(t_ms) if t_ms is not None else None,
                        "x": x,
                        "y": y,
                        "theta": theta,
                        "l_vel": l_vel,
                        "r_vel": r_vel,
                        "speed": speed,
                    }
                )
            except Exception as e:
                print(f"Error parsing log line: {e}")

    return points

def process_watch_data(lines: List[str]) -> List[Dict[str, Any]]:
    """
    WATCH format:
      [WATCH],timestamp_ms,LEVEL,label,value
    We keep label/value as strings. Extra commas in value are preserved by joining.
    """
    events: List[Dict[str, Any]] = []
    for line in lines:
        if "[WATCH]" in line.upper():
            try:
                raw = line.split("[WATCH]", 1)[1].strip()
                parts = [p.strip() for p in raw.split(",")]

                if len(parts) < 4:
                    continue

                t = _try_int(parts[0])
                if t is None:
                    continue

                level = parts[1]
                label = parts[2]
                value = ",".join(parts[3:]).strip()  # preserve commas
                events.append({"t": int(t), "level": level, "label": label, "value": value})
            except Exception as e:
                print(f"Error parsing WATCH line: {e}")
    return events

def apply_range(points: List[Dict[str, Any]], r: Optional[str]) -> List[Dict[str, Any]]:
    if not r:
        return points
    try:
        s, e = map(int, r.split(":"))
        return points[s:e]
    except Exception as e:
        print(f"Error in applying range constraint. {e}")
        return points

def thin_points(points, params):
    """
    Viewer thinning where *movement/turning decides keeps*.
    - Always keep first/last.
    - A point is eligible ONLY if it moved/turned beyond xy_tol/theta_tol vs last kept.
    - If timestamps exist, also require dt >= viewer_thin_ms to keep (rate-limit).
    """
    if not points:
        return [], 0

    min_dt = int(params.get("viewer_thin_ms", 20))

    kept = [points[0]]
    removed = 0
    last = points[0]

    for p in points[1:-1]:
        moved = math.hypot(p["x"] - last["x"], p["y"] - last["y"]) >= params["xy_tol"]
        turned = get_angular_diff(p["theta"], last["theta"]) >= params["theta_tol"]

        if not (moved or turned):
            removed += 1
            continue

        kept.append(p)
        last = p

    kept.append(points[-1])
    return kept, removed

def thin_points_viewer(points: List[Dict[str, Any]], params: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], int]:
    """
    Viewer thinning:
      - If timestamps exist: keep a point if dt >= viewer_thin_ms OR moved/turned beyond xy_tol/theta_tol.
      - Always keep first/last.
    """
    if not points:
        return [], 0

    viewer_thin_ms = int(params.get("viewer_thin_ms", 20))

    kept = [points[0]]
    removed = 0
    last = points[0]

    for p in points[1:-1]:
        dt_ok = False
        if p.get("t") is not None and last.get("t") is not None:
            dt_ok = (p["t"] - last["t"]) >= viewer_thin_ms

        moved = math.hypot(p["x"] - last["x"], p["y"] - last["y"]) >= params["xy_tol"]
        turned = get_angular_diff(p["theta"], last["theta"]) >= params["theta_tol"]

        if dt_ok or moved or turned:
            kept.append(p)
            last = p
        else:
            removed += 1

    kept.append(points[-1])
    return kept, removed

def create_jerryio_json(all_points: List[Dict[str, Any]], name: str, params: Dict[str, Any]):
    # DO NOT CHANGE: per user request, jerryio JSON creation remains identical.
    if not all_points:
        return None, None, None, 0, 0, 0

    # 1. Density Filter
    step1 = [all_points[0]] + all_points[params["density"] :: params["density"]]
    if all_points[-1] not in step1:
        step1.append(all_points[-1])

    # 2. Tolerance Filter (Thinning)
    step2, removed = thin_points(step1, params)

    waypoints = []

    last_v_x = step2[0]["x"]
    last_v_y = step2[0]["y"]
    last_v_theta = step2[0]["theta"]

    vis_xy_tol = params.get("visible_xy_tol", 5.0)
    vis_theta_tol = params.get("visible_theta_tol", 10.0)
    vis_interval = params.get("vis_interval", 15)

    for i, p in enumerate(step2):
        # Distance checks
        dist_since_visible = math.hypot(p["x"] - last_v_x, p["y"] - last_v_y)

        raw_diff = abs(p["theta"] - last_v_theta)
        angle_since_visible = min(raw_diff, 360 - raw_diff)

        # Visibility Logic
        is_edge = (i == 0 or i == len(step2) - 1)
        moved_enough = dist_since_visible >= vis_xy_tol
        turned_enough = angle_since_visible >= vis_theta_tol
        interval_hit = (i % vis_interval == 0)

        should_be_visible = is_edge or moved_enough or turned_enough or interval_hit

        # Append waypoint
        # We do NOT add 'speed' here, so the JSON remains compatible with the website.
        waypoints.append(
            {
                "uid": generate_uid(),
                "x": p["x"],
                "y": p["y"],
                "heading": p["theta"],
                "lock": False,
                "visible": should_be_visible,
                "__type": "end-point",
            }
        )

        if should_be_visible:
            last_v_x, last_v_y, last_v_theta = p["x"], p["y"], p["theta"]

    # 3. Generate Output Strings

    # RAW DATA FOR LemLib: x, y, speed
    # We use .get('speed', 0) to safely handle points that might default to 0
    raw = [f"{p['x']}, {p['y']}, {p.get('speed', 0):.1f}" for p in all_points]

    table = []
    segments = []

    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        mx, my = round((p1["x"] + p2["x"]) / 2, 3), round((p1["y"] + p2["y"]) / 2, 3)

        # Standard midpoint table
        table.append(f"{p1['x']}, {p1['y']}, {mx}, {my}, {mx}, {my}, {p2['x']}, {p2['y']}")

        segments.append(
            {
                "controls": [p1, p2],
                "speedProfiles": [],
                "lookaheadKeyframes": [],
                "uid": generate_uid(),
            }
        )

    json_data = {
        "appVersion": "0.10.0",
        "format": "LemLib v0.5",
        "gc": {
            "robotWidth": params["robot_width"],
            "robotHeight": params["robot_height"],
            "robotIsHolonomic": False,
            "showRobot": False,
            "uol": 2.54,
            "pointDensity": params["jerryio_point_density"],
            "controlMagnetDistance": 1.96,
            "coordinateSystem": "VEX Gaming Positioning System",
        },
        "paths": [
            {
                "segments": segments,
                "pc": {"speedLimit": {"from": 20, "to": 100}, "maxDecelerationRate": 127},
                "name": name,
                "uid": generate_uid(),
                "lock": False,
                "visible": True,
            }
        ],
    }

    return raw, table, json_data, removed, len(step2), sum(w["visible"] for w in waypoints)

def write_output(path: str, raw: List[str], table: List[str], json_obj: Dict[str, Any]) -> None:
    """
    Writes the JerryIO output files.
    - raw: list of raw CSV lines
    - table: list of midpoint/control table lines
    - json_obj: final JSON object for JerryIO
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        f.writelines(l + "\n" for l in raw)
        f.write("endData\n127\n100\n200\n")
        f.writelines(l + "\n" for l in table)
        f.write(f"#PATH.JERRYIO-DATA {json.dumps(json_obj, separators=(',', ':'))}")

def write_viewer_json(path: str, input_name: str, poses: List[Dict[str, Any]], watches: List[Dict[str, Any]], params: Dict[str, Any], thin_removed: int) -> None:
    """
    Writes a clean viewer JSON file for the offline viewer.
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)

    meta = {
        "run_name": input_name,
        "units": params.get("viewer_units", "in"),
        "coord_system": "VEX GPS",
        "robot": {"width": params.get("robot_width", 12), "height": params.get("robot_height", 12)},
        "log_hz": int(params.get("log_hz", 100)),
    }

    out = {
        "format": "jerryforge.viewer",
        "version": 1,
        "meta": meta,
        "thinning": {
            "raw": len(poses) + thin_removed,
            "kept": len(poses),
            "removed": thin_removed,
            "viewer_thin_ms": int(params.get("viewer_thin_ms", 20)),
            "xy_tol": float(params.get("xy_tol", 2.0)),
            "theta_tol": float(params.get("theta_tol", 5.0)),
        },
        "poses": [
            {
                "t": p.get("t"),
                "x": p.get("x"),
                "y": p.get("y"),
                "theta": p.get("theta"),
                "l_vel": p.get("l_vel"),
                "r_vel": p.get("r_vel"),
                "speed": p.get("speed"),
            }
            for p in poses
        ],
        "watches": watches,
    }

    with open(path, "w") as f:
        json.dump(out, f, separators=(",", ":"), ensure_ascii=False)

def run_processor(
    lines: List[str],
    input_name: str,
    params: Dict[str, Any],
    out_file: str,
    path_name: Optional[str] = None,
    range_str: Optional[str] = None,
    verbose: bool = False,
    no_output: bool = False,
    config_file: str = "",
):
    # --- STEP 1: PARSE & STRICT SANITIZATION ---
    raw_points = process_log_data(lines, params)
    watches = process_watch_data(lines)

    def is_valid(p: Dict[str, Any]) -> bool:
        return all(math.isfinite(float(p.get(k, 0))) for k in ["x", "y", "theta"])

    points = [p for p in raw_points if is_valid(p)]
    invalid_count = len(raw_points) - len(points)
    raw_count = len(points)

    if range_str:
        points = apply_range(points, range_str)

    # --- STEP 2: PATH STATISTICS ---
    total_dist = 0.0
    max_x = 0.0
    max_y = 0.0
    optimal_dist = 0.0

    if points:
        max_x = max(p["x"] for p in points)
        max_y = max(p["y"] for p in points)

        for i in range(1, len(points)):
            dist = math.hypot(points[i]["x"] - points[i - 1]["x"], points[i]["y"] - points[i - 1]["y"])
            if math.isfinite(dist):
                total_dist += dist

    # Generate the JerryIO structure 
    raw, table, json_obj, removed, final_wp, visible = create_jerryio_json(points, path_name or input_name, params)

    if json_obj is None or "paths" not in json_obj:
        raise ValueError(f"ERROR: No valid data found in '{input_name}'.")

    # Output switch
    if not no_output:
        if bool(params.get("export_viewer_json", False)):
            # Viewer output (thin separately for viewer playback/heatmaps)
            viewer_points, viewer_removed = thin_points_viewer(points, params)
            write_viewer_json(out_file, input_name, viewer_points, watches, params, viewer_removed)
        else:
            # JerryIO output 
            write_output(out_file, raw, table, json_obj)

    # --- STEP 3: WAYPOINT SPACING (Every waypoint-to-waypoint) ---
    segments = json_obj['paths'][0]['segments']

    spacings = []
    for seg in segments:
        p1, p2 = seg['controls']
        d = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        if math.isfinite(d):
            spacings.append(d)

    min_space = min(spacings) if spacings else 0.0
    max_space = max(spacings) if spacings else 0.0
    avg_space = (sum(spacings) / len(spacings)) if spacings else 0.0


    # --- STEP 4: OPTIMAL DISTANCE (Strict Visible Filter) ---
    all_wps = [segments[0]["controls"][0]] + [seg["controls"][1] for seg in segments]
    visible_wps = [wp for wp in all_wps if wp.get("visible", False)]

    for i in range(len(visible_wps) - 1):
        seg_opt = math.hypot(
            visible_wps[i + 1]["x"] - visible_wps[i]["x"],
            visible_wps[i + 1]["y"] - visible_wps[i]["y"],
        )
        if math.isfinite(seg_opt):
            optimal_dist += seg_opt

    if verbose:
        print("----------| VERBOSE |----------")
        print(f"Path name: {path_name or input_name}")
        print(f"Input file: {os.path.abspath(input_name)}")
        print(f"Output file: {os.path.abspath(out_file)}")
        print(f"Config file: {os.path.abspath(config_file or DEFAULT_CONFIG_FILE)}")

        if invalid_count > 0:
            print(f"[WARNING]: Dropped {invalid_count} points containing NaN/Inf values!")

        print("\n--- Waypoint Spacing ---")
        print(f"Min spacing: {min_space:.2f} units")
        print(f"Max spacing: {max_space:.2f} units")
        print(f"Average spacing: {avg_space:.2f} units")

        print("\n--- Field Bounds ---")
        print(f"Max X: {max_x:.1f}")
        print(f"Max Y: {max_y:.1f}")

        print("\n--- Distance Metrics ---")
        print(f"Distance Traveled: {total_dist:.2f} units")
        print(f"Optimal Travel Distance: {optimal_dist:.2f} units")

        inefficiency = total_dist - optimal_dist
        print(f"Path Inefficiency: {max(0, inefficiency):.2f} units")

        if total_dist > 0 and math.isfinite(total_dist):
            fidelity = (inefficiency / total_dist) * 100
            print(f"Fidelity Loss: {max(0, fidelity):.2f}%")
        else:
            print("Fidelity Loss: 0.00%")

        print("\n--- Thinning Stats (JerryIO) ---")
        print(f"Raw DATA points: {raw_count}")
        print(f"Points Removed (Thinning): {removed}")
        print(f"Final Waypoints: {final_wp}")
        print(f"Visible Waypoints: {visible}")

        if bool(params.get("export_viewer_json", False)):
            print("\n--- Viewer Stats ---")
            print(f"Watches parsed: {len(watches)}")
            print(f"Viewer output enabled: True")

        print("--------------------------------\n")
    else:
        print("\n----------| Success! |----------")
        print(f"File: [{input_name}] Output saved to [{out_file}].")
        print(f"Waypoints: {final_wp}")
        print(f"Visible: {visible}")
        if bool(params.get("export_viewer_json", False)):
            print(f"Watches: {len(watches)}")
        print("--------------------------------\n")

# ==========================================
# REVERSE PARSING
# ==========================================
def cast_jerry_path(input_file: str, output_file: str, config_file: str, verbose: bool = False):
    """
    Cast a JerryIO (LemLib v0.5) file into a templated output.
    """
    if not os.path.exists(input_file):
        raise FileNotFoundError(f"Input file not found: {input_file}")

    try:
        load_config(config_file)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        sys.exit(1)

    if not output_file:
        print("No output filename provided.")
        sys.exit(1)

    # Load config template
    try:
        with open(config_file, "r") as cf:
            cfg = json.load(cf)
    except Exception as e:
        raise ValueError(f"Failed to parse config JSON '{config_file}': {e}")

    if "template" not in cfg or not isinstance(cfg["template"], str):
        raise ValueError("Config JSON must contain a top-level 'template' string field") from None

    template = cfg["template"]

    # Read input file
    with open(input_file, "r") as f:
        raw_lines = [ln.rstrip("\n") for ln in f]

    # find endData line index (case-sensitive 'endData')
    try:
        enddata_idx = raw_lines.index("endData")
    except ValueError as e:
        raise ValueError(f"Input file missing 'endData' marker. Exception: {e}")

    pathdata_lines = raw_lines[:enddata_idx]

    # find JSON in file: line that starts with "#PATH.JERRYIO-DATA"
    json_line = None
    for ln in raw_lines:
        if ln.startswith("#PATH.JERRYIO-DATA"):
            parts = ln.split(" ", 1)
            if len(parts) == 2:
                json_line = parts[1].strip()
            else:
                json_line = ln[len("#PATH.JERRYIO-DATA") :].strip()
            break

    if not json_line:
        raise ValueError("Input file missing '#PATH.JERRYIO-DATA' JSON payload")

    try:
        json_obj = json.loads(json_line)
    except Exception as e:
        raise ValueError(f"Failed to parse JerryIO JSON payload: {e}")

    # Extract default min speed from JSON (fallback to 0.0)
    speed_default = 0.0
    try:
        p0 = (json_obj.get("paths") or [None])[0]
        if p0 and "pc" in p0 and "speedLimit" in p0["pc"]:
            sl = p0["pc"]["speedLimit"]
            if isinstance(sl, dict):
                if "from" in sl and isinstance(sl["from"], (int, float)):
                    speed_default = float(sl["from"])
                elif "minLimit" in sl and isinstance(sl["minLimit"], dict) and "value" in sl["minLimit"]:
                    speed_default = float(sl["minLimit"]["value"])
    except Exception:
        speed_default = 0.0

    # Build waypoint array from JSON segments -> controls
    waypoints: List[Dict[str, Any]] = []
    seen_uids = set()

    with open(config_file, "r") as f:
        params = json.load(f)

    render_invisible = params.get("render_invisible_waypoints", False)

    try:
        paths = json_obj.get("paths") or []
        for path in paths:
            segments = path.get("segments") or []
            for seg in segments:
                controls = seg.get("controls") or []
                for ctrl in controls:
                    uid = ctrl.get("uid")
                    if uid is None:
                        uid = f"coord:{ctrl.get('x')}:{ctrl.get('y')}:{ctrl.get('heading')}"
                    if uid in seen_uids:
                        continue
                    seen_uids.add(uid)

                    try:
                        x = float(ctrl.get("x", 0.0))
                        y = float(ctrl.get("y", 0.0))
                        theta = float(ctrl.get("heading", 0.0))
                        visible = bool(ctrl.get("visible", True))
                    except Exception:
                        raise ValueError(f"Invalid control values for waypoint UID {uid}: {ctrl}")

                    if not render_invisible and not visible:
                        continue

                    waypoints.append(
                        {"x": x, "y": y, "theta": theta, "uid": uid, "speed": float(speed_default), "visible": visible}
                    )

    except Exception as e:
        raise ValueError(f"Failed to extract waypoints from JSON: {e}")

    if not waypoints:
        raise ValueError("No waypoints extracted from JerryIO JSON")

    # Parse pathData array (x,y,speed) from top of file
    pathData: List[Dict[str, Any]] = []
    for ln in pathdata_lines:
        s = ln.strip()
        if not s:
            continue
        parts = [p.strip() for p in s.split(",") if p.strip() != ""]
        if len(parts) < 3:
            continue
        try:
            px = float(parts[0])
            py = float(parts[1])
            ps = float(parts[2])
            pathData.append({"x": px, "y": py, "speed": ps})
        except ValueError:
            continue

    if not pathData:
        raise ValueError("No valid pathData (x,y,speed) found before endData in the input file")
        sys.exit(1)

    # Link speeds to waypoints (first unused matching x,y within tolerance)
    used_path_indices: List[int] = []
    assigned_count = 0

    coord_map = {}
    for idx, pd in enumerate(pathData):
        key = (pd["x"], pd["y"])
        coord_map.setdefault(key, []).append(idx)

    for w in waypoints:
        found_speed = None
        for idx, pd in enumerate(pathData):
            if idx in used_path_indices:
                continue
            if abs(w["x"] - pd["x"]) <= COORD_TOLERANCE and abs(w["y"] - pd["y"]) <= COORD_TOLERANCE:
                used_path_indices.append(idx)
                found_speed = pd["speed"]
                break

        if found_speed is not None:
            w["speed"] = float(found_speed)
            assigned_count += 1
        else:
            # fallback: try exact key map
            key = (w["x"], w["y"])
            if key in coord_map:
                for idx in coord_map[key]:
                    if idx not in used_path_indices:
                        used_path_indices.append(idx)
                        w["speed"] = float(pathData[idx]["speed"])
                        assigned_count += 1
                        break

    # Summary
    total_waypoints = len(waypoints)
    total_pathdata = len(pathData)
    unmatched_waypoints = total_waypoints - assigned_count

    print("\n----------| Cast Summary |----------")
    print(f"Input file: {input_file}")
    print(f"Output file: {output_file}")
    print(f"Template config: {config_file}")
    print(f"Waypoints extracted: {total_waypoints}")
    print(f"PathData rows parsed: {total_pathdata}")
    print(f"Speeds assigned to waypoints: {assigned_count}")
    print(f"Waypoints without matched speed (use default): {unmatched_waypoints}")
    print(f"PathData indexes used: {len(used_path_indices)}")
    print("--------------------------------------\n")

    if verbose:
        print("\n-- Verbose details --")
        print("Waypoints (first 10):")
        for i, w in enumerate(waypoints[:10], start=1):
            print(f" {i}: uid={w['uid']} x={w['x']} y={w['y']} theta={w['theta']} speed={w['speed']}")
        print("-- End verbose --\n")

    # Render template for each waypoint and write output file
    out_lines = []
    prev_x = None
    prev_y = None

    for i, w in enumerate(waypoints, start=1):
        x = w["x"]
        y = w["y"]
        theta = w["theta"]
        speed = w["speed"]
        distance = 0.0 if prev_x is None else math.hypot(x - prev_x, y - prev_y)
        prev_x, prev_y = x, y

        line = template
        line = line.replace("${x}", format(x, ".6f").rstrip("0").rstrip(".") if isinstance(x, float) else str(x))
        line = line.replace("${y}", format(y, ".6f").rstrip("0").rstrip(".") if isinstance(y, float) else str(y))
        line = line.replace("${theta}", format(theta, ".6f").rstrip("0").rstrip(".") if isinstance(theta, float) else str(theta))
        line = line.replace("${speed}", format(speed, ".6f").rstrip("0").rstrip(".") if isinstance(speed, float) else str(speed))
        line = line.replace("${iteration}", str(i))
        line = line.replace("${distance}", format(distance, ".6f").rstrip("0").rstrip("."))

        out_lines.append(line)

    with open(DEFAULT_CONFIG_FILE, "r") as f:
        default_params = json.load(f)

    output_folder = default_params.get("output_folder", "./") or "./"
    output_filename = os.path.basename(output_file)
    full_output_path = os.path.join(output_folder, output_filename)

    os.makedirs(output_folder, exist_ok=True)

    with open(full_output_path, "w") as of:
        for ln in out_lines:
            if not isinstance(ln, str):
                raise ValueError(f"Invalid line type: {type(ln)}. Must be str.")
            of.write(ln + ("\n" if not ln.endswith("\n") else ""))

    print(f"[OK] Wrote {len(out_lines)} template lines to '{full_output_path}'")

# ==========================================
# MAIN
# ==========================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch", action="store_true", help="--batch [OPTIONAL ARGUMENTS]. Used for processing multiple log files at the same time.")
    parser.add_argument("--latest", action="store_true", help="--latest [OPTIONAL ARGUMENTS]. Used for processing the most new file (default).")
    parser.add_argument("--paste", action="store_true", help="--paste [OPTIONAL ARGUMENTS]. Used typically for smaller logs. Type 'done' on a new line when finished.")
    parser.add_argument("--sort", choices=["newest", "oldest"], help="--sort newest|oldest [OPTIONAL ARGUMENTS]. Used for sorting all files found in json destinations.")
    parser.add_argument("--file", help="--file <filename> [OPTIONAL ARGUMENTS]. Used for processing a give file name.")
    # Exclusive for --batch
    parser.add_argument("--skip-confirm", action="store_true", help="--skip-confirm. Used with --batch only. Skips all file confirmation prompts.")
    parser.add_argument("--new", type=int, help="--new <int>. Used with --batch. Uses the n most recent files.")
    parser.add_argument("--old", type=int, help="--old <int>. Used with --batch. Uses the n oldest files.")
    # General flags
    parser.add_argument("--range", help="--range start:end. Only outputs points from start to end")
    parser.add_argument("--out", help="--out <name>. Allows overriding of the default file name.")
    parser.add_argument("--path-name", help="--path-name <name>. Allows overriding of the default JerryIO Path name")
    parser.add_argument("--config", help="--config <json_filename>. Allows changing of the config file without changing any text")
    parser.add_argument("--fast", action="store_true", help="--fast. Allows skipping of all questions. Defaults are used, and all files are accepted")
    parser.add_argument("--verbose", action="store_true", help="--verbose. Prints more information about the path created")
    parser.add_argument("--no-output", action="store_true", help="--no-output. The provided command will not produce an output file. Used for statistics")
    # Exclusive for --latest
    parser.add_argument("--index", type=int, help="--index <int>. Used with --latest. Used for getting a specific index of the latest files.")
    # For JerryIO Reverse parsing
    parser.add_argument("--cast", action="store_true", help="--cast --out <name> --file <filename> --config <jsonfile> [OPTIONAL] Convert a JerryIO file into templated autonomous code")

    args = parser.parse_args()
    CONFIG = load_config(args.config or DEFAULT_CONFIG_FILE)

    # Reverse Conversion: --cast
    if args.cast:
        input_file = None

        if args.file:
            if not os.path.exists(args.file):
                raise FileNotFoundError(f"Specified file not found: {args.file}")
            input_file = args.file

        elif args.latest:
            json_path = args.config or DEFAULT_CONFIG_FILE
            with open(json_path, "r") as f:
                cfg = json.load(f)

            scan_paths = cfg.get("scan_paths_latest", [])
            if not scan_paths:
                raise FileNotFoundError("No destinations in config: scan_paths_latest")

            txt_files = []
            for p in scan_paths:
                if os.path.isdir(p):
                    for f_name in os.listdir(p):
                        if f_name.endswith(".txt"):
                            txt_files.append(os.path.join(p, f_name))
                elif os.path.isfile(p) and p.endswith(".txt"):
                    txt_files.append(p)
                else:
                    if args.verbose:
                        print(f"[WARN] Ignoring invalid path: {p}")

            if not txt_files:
                raise FileNotFoundError("No .txt files found in scan_paths_latest directories/files")

            txt_files.sort(key=lambda f: os.path.getmtime(f), reverse=(args.sort != "oldest"))

            if args.fast:
                input_file = txt_files[0]
                print(f"[FAST] Using latest file: {input_file}")
            else:
                for fpath in txt_files:
                    if confirm(f"Use this file for conversion: {fpath}?"):
                        input_file = fpath
                        break
                if input_file is None:
                    raise RuntimeError("No file selected for conversion with --latest")
        else:
            raise ValueError("You must specify either --file <filename> or --latest with --cast")

        try:
            cast_jerry_path(
                input_file=input_file,
                output_file=args.out,
                config_file=args.config or DEFAULT_CONFIG_FILE,
                verbose=args.verbose,
            )
        except (ValueError, FileNotFoundError) as e:
            print(f"[ERROR] {e}")
            sys.exit(1)
        sys.exit(0)

    # ---------- VALIDATION ----------
    if not (args.batch or args.latest or args.file or args.paste or args.cast):
        parser.error("Must specify a command")

    if not args.batch and (args.skip_confirm or args.new or args.old):
        parser.error("Batch-only flags used without --batch")

    if args.new and args.old:
        parser.error("--new and --old cannot be used together")

    params = CONFIG if args.fast else prompt_for_config(CONFIG)
    base_out = ensure_output_folder(params["output_folder"])

    # Choose output suffix based on config (viewer vs jerryio)
    def make_outfile(base_path: str) -> str:
        if bool(params.get("export_viewer_json", False)):
            return base_path + "-viewer.json"
        return base_path + "-jerryio.txt"

    # ---------- BATCH ----------
    if args.batch:
        try:
            logs = get_logs_from_paths(params["scan_paths_batch"])
            logs = sort_logs(logs, newest=not args.old)
            if args.new:
                logs = logs[: args.new]
            if args.old:
                logs = logs[: args.old]

            if not logs:
                print("No .log files found in scan_paths_batch")
                return

            folder = args.out or args.path_name or os.path.splitext(os.path.basename(logs[0]))[0]
            batch_dir = os.path.join(base_out, folder)
            os.makedirs(batch_dir, exist_ok=True)

            for log in logs:
                if not args.skip_confirm and not args.fast and not confirm(f"Process {log}?"):
                    continue
                with open(log) as f:
                    base = os.path.splitext(os.path.basename(log))[0]
                    out_base = os.path.join(batch_dir, sanitize_batch_name(args.out, base))
                    run_processor(
                        f.readlines(),
                        base,
                        params,
                        make_outfile(out_base),
                        path_name=sanitize_batch_name(args.path_name, base),
                        range_str=args.range,
                        verbose=args.verbose,
                        no_output=args.no_output,
                        config_file=args.config or DEFAULT_CONFIG_FILE,
                    )

        except (ValueError, FileNotFoundError, IOError) as e:
            print(f"[ERROR][batch] {e}")
            sys.exit(1)

    # ---------- LATEST ----------
    elif args.latest:
        try:
            logs = sort_logs(get_logs_from_paths(params["scan_paths_latest"]), newest=True)
            if not logs:
                print("No .log files found in scan_paths_latest")
                return

            if args.index is not None:
                if args.index < 0 or args.index >= len(logs):
                    print(f"Error: --index {args.index} out of range (0-{len(logs)-1})")
                    return
                logs = [logs[args.index]]

            log = logs[0]
            if not args.fast and not confirm(f"Use {log}?"):
                print("Aborted!")
                sys.exit(1)

            with open(log) as f:
                base = os.path.splitext(os.path.basename(log))[0]
                out_base = os.path.join(base_out, args.out or base)
                run_processor(
                    f.readlines(),
                    base,
                    params,
                    make_outfile(out_base),
                    path_name=sanitize_batch_name(args.path_name, base),
                    range_str=args.range,
                    verbose=args.verbose,
                    no_output=args.no_output,
                    config_file=args.config or DEFAULT_CONFIG_FILE,
                )

        except (ValueError, FileNotFoundError, IOError) as e:
            print(f"[ERROR][latest] {e}")
            sys.exit(1)

    # ---------- FILE ----------
    elif args.file:
        try:
            if not os.path.exists(args.file):
                raise FileNotFoundError(f"Specified file not found: {args.file}")

            if not args.fast and not confirm(f"Use {args.file}?"):
                print("Aborted!")
                sys.exit(1)

            with open(args.file) as f:
                base = os.path.splitext(os.path.basename(args.file))[0]
                out_base = os.path.join(base_out, args.out or base)
                run_processor(
                    f.readlines(),
                    base,
                    params,
                    make_outfile(out_base),
                    path_name=sanitize_batch_name(args.path_name, base),
                    range_str=args.range,
                    verbose=args.verbose,
                    no_output=args.no_output,
                    config_file=args.config or DEFAULT_CONFIG_FILE,
                )

        except (ValueError, FileNotFoundError, IOError) as e:
            print(f"[ERROR][file] {e}")
            sys.exit(1)

    # ---------- PASTE ----------
    elif args.paste:
        try:
            print("Paste data, then type 'done'")
            lines = []
            while True:
                line = input()
                if line.lower() == "done":
                    break
                lines.append(line)

            pts = apply_range(process_log_data(lines, params), args.range)
            watches = process_watch_data(lines)

            raw, table, json_obj, removed, final_wp, visible = create_jerryio_json(
                pts, args.path_name or "Pasted_Path", params
            )

            if not args.no_output:
                out_base = os.path.join(base_out, "Pasted_Path")
                if bool(params.get("export_viewer_json", False)):
                    viewer_points, viewer_removed = thin_points_viewer(pts, params)
                    write_viewer_json(make_outfile(out_base), "Pasted_Path", viewer_points, watches, params, viewer_removed)
                else:
                    write_output(make_outfile(out_base), raw, table, json_obj)

        except (ValueError, IOError) as e:
            print(f"[ERROR][paste] {e}")
            sys.exit(1)

if __name__ == "__main__":
    main()
