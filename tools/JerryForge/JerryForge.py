#!/usr/bin/env python3
import json, secrets, string, os, math, sys, glob, argparse, re, time
from typing import List, Dict

# ==========================================
# CONFIGURATION
# ==========================================
DEFAULT_CONFIG_FILE = 'jerry_plotter-defaults.json'
COORD_TOLERANCE = 1  # 

def load_config(file_path=DEFAULT_CONFIG_FILE):
    fallback = {
        # Thinning
        'density': 1,
        'xy_tol': 2.0,
        'theta_tol': 5.0,
        # Visuals 
       'visible_xy_tol': 5.0,
       'visible_theta_tol': 10.0, 
        'vis_interval': 1000,
        # Ease of use
        'offset_x': 0.0,
        'offset_y': 0.0,
        'offset_theta': 0.0,
        # Quality of life and visual settings
        'robot_width': 12,
        'robot_height': 12,
        'jerryio_point_density': 2,
        # File searching locations
        'scan_paths_latest': ["./"],
        'scan_paths_batch': ["./"],
        'output_folder': "",
        # PathCaster
        'render_invisible_waypoints': False,
        'template': ""
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
def get_logs_from_paths(paths):
    logs = []
    for p in paths:
        logs.extend(glob.glob(os.path.join(os.path.expanduser(p), "*.log")))
    return logs

def sort_logs(logs, newest=True):
    return sorted(logs, key=os.path.getmtime, reverse=newest)

def generate_uid():
    return ''.join(secrets.choice(string.ascii_letters + string.digits) for _ in range(10))

def normalize_heading(theta):
    return round((theta % 360 + 360) % 360, 3)

def get_angular_diff(a, b):
    diff = abs(a - b) % 360
    return 360 - diff if diff > 180 else diff

def confirm(msg):
    return input(f"{msg} (y/n): ").strip().lower() == 'y'

def sanitize_batch_name(base, original):
    return f"{base}_{original}" if base else original

def ensure_output_folder(path):
    if not path or re.search(r'[*:?<>|]', path):
        return "./"
    os.makedirs(path, exist_ok=True)
    return path

def prompt_for_config(defaults):
    print("\nConfigure Path Parameters (Enter = default)")
    return {
        'density': int(input(f"Path Density [{defaults['density']}]: ") or defaults['density']),
        'xy_tol': float(input(f"XY Tolerance [{defaults['xy_tol']}]: ") or defaults['xy_tol']),
        'theta_tol': float(input(f"Theta Tolerance [{defaults['theta_tol']}]: ") or defaults['theta_tol']),
        'vis_interval': int(input(f"Visibility Interval [{defaults['vis_interval']}]: ") or defaults['vis_interval']),
        'offset_x': float(input(f"Offset X [{defaults['offset_x']}]: ") or defaults['offset_x']),
        'offset_y': float(input(f"Offset Y [{defaults['offset_y']}]: ") or defaults['offset_y']),
        'offset_theta': float(input(f"Offset Theta [{defaults['offset_theta']}]: ") or defaults['offset_theta']),
        'robot_width': defaults['robot_width'],
        'robot_height': defaults['robot_height'],
        'jerryio_point_density': defaults['jerryio_point_density'],
        'scan_paths_latest': defaults['scan_paths_latest'],
        'scan_paths_batch': defaults['scan_paths_batch'],
        'output_folder': defaults['output_folder']
    }

# ==========================================
# LOG PROCESSING 
# ==========================================
def process_log_data(lines, params):
    points = []
    for line in lines:
        if "[DATA]" in line.upper():
            try:
                raw = line.upper().split("[DATA]")[1].strip()
                parts = [p.strip() for p in raw.split(',') if p.strip()]
                # Verify that data line contains 3 values
                if len(parts) >= 3:
                    points.append({
                        'x': round(float(parts[0]) + params['offset_x'], 3),
                        'y': round(float(parts[1]) + params['offset_y'], 3),
                        'theta': normalize_heading(float(parts[2]) + params['offset_theta'])
                    })
            except Exception as e:
                print(f"Error occurred in parsing log data. Ignoring point. {e}")
    return points

def apply_range(points, r):
    if not r:
        return points
    try:
        s, e = map(int, r.split(':'))
        return points[s:e]
    except Exception as e:
        print(f"Error in applying range constraint. {e}")
        return points

def thin_points(points, params):
    if not points:
        return [], 0
    kept = [points[0]]
    removed = 0
    last = points[0]
    for p in points[1:-1]:
        if (math.hypot(p['x'] - last['x'], p['y'] - last['y']) >= params['xy_tol']
            or get_angular_diff(p['theta'], last['theta']) >= params['theta_tol']):
            kept.append(p)
            last = p
        else:
            removed += 1
    kept.append(points[-1])
    return kept, removed

def create_jerryio_json(all_points, name, params):
    if not all_points:
        return None, None, None, 0, 0, 0

    step1 = [all_points[0]] + all_points[params['density']::params['density']]
    if all_points[-1] not in step1:
        step1.append(all_points[-1])

    step2, removed = thin_points(step1, params)

    waypoints = []
    
    # Track the last point in the loop (for step-by-step distance)
    last_p_x = step2[0]['x']
    last_p_y = step2[0]['y']
    last_p_theta = step2[0]['theta']
    
    # Track the last VISIBLE point (to measure when to trigger the next one)
    last_v_x = step2[0]['x']
    last_v_y = step2[0]['y']
    last_v_theta = step2[0]['theta']
    
    vis_xy_tol = params.get('visible_xy_tol', 5.0)
    vis_theta_tol = params.get('visible_theta_tol', 10.0)
    vis_interval = params.get('vis_interval', 15)

    for i, p in enumerate(step2):
        # 1. Distance between THIS point and the PREVIOUS point in step2
        # (This is the 'regardless of visibility' distance you asked for)
        step_dist = math.hypot(p['x'] - last_p_x, p['y'] - last_p_y)
        
        # 2. Accumulated distance since the last VISIBLE waypoint
        # We check this to see if we've moved enough to drop a new marker
        dist_since_visible = math.hypot(p['x'] - last_v_x, p['y'] - last_v_y)
        
        # 3. Angular difference since the last VISIBLE waypoint
        raw_diff = abs(p['theta'] - last_v_theta)
        angle_since_visible = min(raw_diff, 360 - raw_diff)

        # Visibility Conditions
        is_edge = (i == 0 or i == len(step2) - 1)
        moved_enough = dist_since_visible >= vis_xy_tol
        turned_enough = angle_since_visible >= vis_theta_tol
        interval_hit = (i % vis_interval == 0)

        should_be_visible = is_edge or moved_enough or turned_enough or interval_hit

        waypoints.append({
            'uid': generate_uid(),
            'x': p['x'], 
            'y': p['y'],
            'heading': p['theta'],
            'lock': False,
            'visible': should_be_visible,
            '__type': 'end-point'
        })

        # ALWAYS update the last point to the current one for the next iteration
        last_p_x, last_p_y, last_p_theta = p['x'], p['y'], p['theta']

        # ONLY update the last VISIBLE anchor when a marker is actually placed
        if should_be_visible:
            last_v_x, last_v_y, last_v_theta = p['x'], p['y'], p['theta']

    raw = [f"{p['x']}, {p['y']}, {p['theta']}" for p in all_points]
    table = []
    segments = []

    for i in range(len(waypoints)-1):
        p1, p2 = waypoints[i], waypoints[i+1]
        mx, my = round((p1['x']+p2['x'])/2,3), round((p1['y']+p2['y'])/2,3)
        table.append(f"{p1['x']}, {p1['y']}, {mx}, {my}, {mx}, {my}, {p2['x']}, {p2['y']}")
        segments.append({'controls':[p1,p2],'speedProfiles':[],'lookaheadKeyframes':[],'uid':generate_uid()})

    json_data = {
        "appVersion":"0.10.0",
        "format":"LemLib v0.5",
        "gc":{
            "robotWidth":params['robot_width'],
            "robotHeight":params['robot_height'],
            "robotIsHolonomic":False,
            "showRobot":False,
            "uol":2.54,
            "pointDensity":params['jerryio_point_density'],
            "controlMagnetDistance":1.96,
            "coordinateSystem":"VEX Gaming Positioning System"
        },
        "paths":[{
            "segments":segments,
            "pc":{"speedLimit":{"from":20,"to":100},"maxDecelerationRate":127},
            "name":name,
            "uid":generate_uid(),
            "lock":False,
            "visible":True
        }]
    }

    return raw, table, json_data, removed, len(step2), sum(w['visible'] for w in waypoints)

def write_output(path, raw, table, json_obj):
    """
    Writes the JerryIO output files.
    - raw: list of raw CSV lines
    - table: list of midpoint/control table lines
    - json_obj: final JSON object for JerryIO
    """
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, 'w') as f:
        f.writelines(l + "\n" for l in raw)
        f.write("endData\n127\n100\n200\n")
        f.writelines(l + "\n" for l in table)
        f.write(f"#PATH.JERRYIO-DATA {json.dumps(json_obj, separators=(',', ':'))}")

def run_processor(lines, input_name, params, out_file,
                  path_name=None, range_str=None,
                  verbose=False, no_output=False, config_file=""):

    # --- STEP 1: PARSE & STRICT SANITIZATION ---
    raw_points = process_log_data(lines, params)
    
    # Validation function to ensure no coordinate is NaN or Inf
    def is_valid(p):
        # Check all three critical axes
        return all(math.isfinite(p.get(k, 0)) for k in ['x', 'y', 'theta'])

    # Filter points immediately
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
        # Bounding Box logic (safely handles clean data)
        max_x = max(p['x'] for p in points)
        max_y = max(p['y'] for p in points)
        
        # Odometer logic (Distance Traveled)
        for i in range(1, len(points)):
            dist = math.hypot(
                points[i]['x'] - points[i-1]['x'], 
                points[i]['y'] - points[i-1]['y']
            )
            # One last safety check just in case hypot fails
            if math.isfinite(dist):
                total_dist += dist

    # Generate the JerryIO structure
    raw, table, json_obj, removed, final_wp, visible = \
        create_jerryio_json(points, path_name or input_name, params)

    if json_obj is None or 'paths' not in json_obj:
        raise ValueError(f"ERROR: No valid data found in '{input_name}'.")

    if not no_output:
        write_output(out_file, raw, table, json_obj)

    # --- STEP 3: WAYPOINT SPACING (Visible to Visible) ---
    spacings = []
    current_path_dist = 0
    segments = json_obj['paths'][0]['segments']
    
    for seg in segments:
        p1, p2 = seg['controls']
        seg_dist = math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
        
        if math.isfinite(seg_dist):
            current_path_dist += seg_dist
        
        if p2.get('visible', False):
            if current_path_dist > 0:
                spacings.append(current_path_dist)
            current_path_dist = 0

    min_space = min(spacings) if spacings else 0
    max_space = max(spacings) if spacings else 0
    avg_space = sum(spacings) / len(spacings) if spacings else 0

    # --- STEP 4: OPTIMAL DISTANCE (Strict Visible Filter) ---
    all_wps = [segments[0]['controls'][0]] + [seg['controls'][1] for seg in segments]
    visible_wps = [wp for wp in all_wps if wp.get('visible', False)]
    
    for i in range(len(visible_wps) - 1):
        seg_opt = math.hypot(visible_wps[i+1]['x'] - visible_wps[i]['x'], 
                             visible_wps[i+1]['y'] - visible_wps[i]['y'])
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

        print(f"\n--- Waypoint Spacing ---")
        print(f"Min spacing: {min_space:.2f} units")
        print(f"Max spacing: {max_space:.2f} units")
        print(f"Average spacing: {avg_space:.2f} units")

        print(f"\n--- Field Bounds ---")
        print(f"Max X: {max_x:.1f}")
        print(f"Max Y: {max_y:.1f}")
        
        print(f"\n--- Distance Metrics ---")
        # Ensure we don't print 'nan' even if somehow a zero-division occurs
        print(f"Distance Traveled: {total_dist:.2f} units")
        print(f"Optimal Travel Distance: {optimal_dist:.2f} units")
        
        inefficiency = total_dist - optimal_dist
        print(f"Path Inefficiency: {max(0, inefficiency):.2f} units")
        
        if total_dist > 0 and math.isfinite(total_dist):
            fidelity = (inefficiency / total_dist) * 100
            print(f"Fidelity Loss: {max(0, fidelity):.2f}%")
        else:
            print(f"Fidelity Loss: 0.00%")
        
        print(f"\n--- Thinning Stats ---")
        print(f"Raw DATA points: {raw_count}")
        print(f"Points Removed (Thinning): {removed}")
        print(f"Final Waypoints: {final_wp}")
        print(f"Visible Waypoints: {visible}")
        print("--------------------------------\n")
    else:
        print("\n----------| Success! |----------")
        print(f"File: [{input_name}] Output saved to [{out_file}].")
        print(f"Waypoints: {final_wp}")
        print(f"Visible: {visible}")
        print("--------------------------------\n")

# ==========================================
# REVERSE PARSING
# ==========================================

def cast_jerry_path(input_file: str, output_file: str, config_file, verbose: bool = False):
    """
    Cast a JerryIO (LemLib v0.5) file into a templated output.

    Behavior (per spec):
      - CLI syntax handled externally: --cast --file <filename> --out <outfilename> --config <jsonfile>
      - Build waypoint array: [x (float), y (float), theta (float), speed (float), uid (string)]
        * Initially fill x,y,theta,uid; speed defaults to the min speed param found in the jerryio JSON
        * Once a UID has been used, later controls with same UID are skipped
      - Build pathData: lines from file start up to "endData" that parse as (x,y,speed)
      - Match each waypoint to the first unused pathData entry with same (x,y); mark that index used
      - Print summary (supports verbose)
      - Expand template (config JSON must contain "template": "<...>") and write output file
      - Substitutions available: ${x}, ${y}, ${theta}, ${speed}, ${iteration}, ${distance}
    """

    # -------------------------
    # Validation
    # -------------------------
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

    # -------------------------
    # Load config template
    # -------------------------
    try:
        with open(config_file, "r") as cf:
            cfg = json.load(cf)
    except Exception as e:
        raise ValueError(f"Failed to parse config JSON '{config_file}': {e}")

    if "template" not in cfg or not isinstance(cfg["template"], str):
        raise ValueError(
            "Config JSON must contain a top-level 'template' string field"
        ) from None

    template = cfg["template"]

    # -------------------------
    # Read input file
    # -------------------------
    with open(input_file, "r") as f:
        raw_lines = [ln.rstrip("\n") for ln in f]

    # find endData line index (case-sensitive 'endData')
    try:
        enddata_idx = raw_lines.index("endData")
    except ValueError as e:
        raise ValueError(f"Input file missing 'endData' marker. Exception: {e}")

    # pathData lines: from start up to (but not including) endData
    pathdata_lines = raw_lines[:enddata_idx]

    # find JSON in file: line that starts with "#PATH.JERRYIO-DATA"
    json_line = None
    for ln in raw_lines:
        if ln.startswith("#PATH.JERRYIO-DATA"):
            # everything after the space is the JSON blob
            parts = ln.split(" ", 1)
            if len(parts) == 2:
                json_line = parts[1].strip()
            else:
                # maybe no space, take remainder
                json_line = ln[len("#PATH.JERRYIO-DATA"):].strip()
            break

    if not json_line:
        raise ValueError("Input file missing '#PATH.JERRYIO-DATA' JSON payload")

    # parse JSON object
    try:
        json_obj = json.loads(json_line)
    except Exception as e:
        raise ValueError(f"Failed to parse JerryIO JSON payload: {e}")

    # -------------------------
    # Extract default min speed from JSON (fallback to 0.0)
    # Try common locations like: paths[0].pc.speedLimit.from  OR pc.speedLimit.from
    # -------------------------
    speed_default = 0.0
    try:
        p0 = (json_obj.get("paths") or [None])[0]
        if p0 and "pc" in p0 and "speedLimit" in p0["pc"]:
            sl = p0["pc"]["speedLimit"]
            # prefer 'from' if present
            if isinstance(sl, dict):
                if "from" in sl and isinstance(sl["from"], (int, float)):
                    speed_default = float(sl["from"])
                elif "minLimit" in sl and isinstance(sl["minLimit"], dict) and "value" in sl["minLimit"]:
                    speed_default = float(sl["minLimit"]["value"])
            # else leave default
    except Exception:
        speed_default = 0.0

    # -------------------------
    # Build waypoint array from JSON segments -> controls
    # - keep order
    # - only add control if its uid not seen yet
    # Each waypoint entry: dict with x,y,theta,uid,speed (fill speed later)
    # -------------------------
    waypoints: List[Dict] = []
    seen_uids = set()

    with open(config_file, 'r') as f:
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
                    # If uid missing, derive a synthetic uid from coordinates (less ideal)
                    if uid is None:
                        uid = f"coord:{ctrl.get('x')}:{ctrl.get('y')}:{ctrl.get('heading')}"
                    if uid in seen_uids:
                        continue
                    seen_uids.add(uid)
                    
                    # read numeric x, y, heading
                    try:
                        x = float(ctrl.get("x", 0.0))
                        y = float(ctrl.get("y", 0.0))
                        theta = float(ctrl.get("heading", 0.0))
                        visible = bool(ctrl.get("visible", True))  # default to True if not specified
                    except Exception:
                        raise ValueError(f"Invalid control values for waypoint UID {uid}: {ctrl}")
                    
                    # skip waypoint if invisible and not rendering invisible points
                    if not render_invisible and not visible:
                        continue

                    waypoints.append({
                        "x": x,
                        "y": y,
                        "theta": theta,
                        "uid": uid,
                        "speed": float(speed_default),  # default until overwritten
                        "visible": visible  # store for potential future use
                    })

    except Exception as e:
        raise ValueError(f"Failed to extract waypoints from JSON: {e}")

    if not waypoints:
        raise ValueError("No waypoints extracted from JerryIO JSON")

    # -------------------------
    # Parse pathData array (x,y,speed) from top of file
    # Only lines that look like three comma-separated numbers are considered
    # -------------------------
    pathData: List[Dict] = []
    for ln in pathdata_lines:
        s = ln.strip()
        if not s:
            continue
        # Try to parse csv with floats; allow spaces
        parts = [p.strip() for p in s.split(",") if p.strip() != ""]
        if len(parts) < 3:
            continue
        # the first three tokens are x,y,speed (others ignored)
        try:
            px = float(parts[0])
            py = float(parts[1])
            ps = float(parts[2])
            pathData.append({"x": px, "y": py, "speed": ps})
        except ValueError:
            # not a numeric line; skip
            continue

    if not pathData:
        raise ValueError("No valid pathData (x,y,speed) found before endData in the input file")
        sys.exit(1)

    # -------------------------
    # Link speeds to waypoints using used_path_indices list (prevents reuse)
    # For each waypoint in order, find the first pathData index with same (x,y)
    # that is not yet used. Assign that speed and mark index as used.
    # -------------------------
    used_path_indices: List[int] = []
    assigned_count = 0

    # To speed up lookup, build coord -> list[indexes] map
    coord_map = {}
    for w in waypoints:
        found_speed = None

        for idx, pd in enumerate(pathData):
            if idx in used_path_indices:
                continue

            if (
                abs(w["x"] - pd["x"]) <= COORD_TOLERANCE and
                abs(w["y"] - pd["y"]) <= COORD_TOLERANCE
            ):
                used_path_indices.append(idx)
                found_speed = pd["speed"]
                break

        if found_speed is not None:
            w["speed"] = float(found_speed)
            assigned_count += 1
        else:
            w["speed"] = float(w.get("speed", speed_default))
            key = (w["x"], w["y"])
            found_speed = None
            if key in coord_map:
                for idx in coord_map[key]:
                    if idx not in used_path_indices:
                        used_path_indices.append(idx)
                        found_speed = pathData[idx]["speed"]
                        break
            if found_speed is not None:
                w["speed"] = float(found_speed)
                assigned_count += 1
            else:
                # leave default (speed_default)
                w["speed"] = float(w.get("speed", speed_default))

    # -------------------------
    # Summary / verbose output
    # -------------------------
    total_waypoints = len(waypoints)
    total_pathdata = len(pathData)
    unused_pathdata_count = total_pathdata - len(used_path_indices)
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
        print("\nUsed pathData indices (first 100):")
        print(sorted(used_path_indices)[:100])
        print("-- end verbose --\n")

    # -------------------------
    # Render template for each waypoint and write output file
    # Substitutions: ${x}, ${y}, ${theta}, ${speed}, ${iteration}, ${distance}
    # Distance computed from previous waypoint (0 for first)
    # -------------------------
    out_lines = []
    prev_x = None
    prev_y = None

    for i, w in enumerate(waypoints, start=1):
        x = w["x"]
        y = w["y"]
        theta = w["theta"]
        speed = w["speed"]
        if prev_x is None:
            distance = 0.0
        else:
            distance = math.hypot(x - prev_x, y - prev_y)
        prev_x, prev_y = x, y

        # Simple substitution; treat values as floats - render with minimal formatting
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

    # Determine output folder: default to "." if not specified
    output_folder = default_params.get("output_folder", "./")
    if output_folder == ".":
        print("output_folder field is blank in default json. Defaulting to working directory. \n")
    # If user passed a full output_file, keep its filename but prepend the folder
    output_filename = os.path.basename(output_file)
    full_output_path = os.path.join(output_folder, output_filename)

    # Ensure output folder exists
    os.makedirs(output_folder, exist_ok=True)

    # Write final output lines
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
    parser.add_argument('--batch', action='store_true', help="--batch [OPTIONAL ARGUMENTS]. Used for processing multiple log files at the same time.")
    parser.add_argument('--latest', action='store_true', help="--latest [OPTIONAL ARGUMENTS]. Used for processing the most recent file (default).")
    parser.add_argument('--paste', action='store_true', help="--paste [OPTIONAL ARGUMENTS]. Used typically for smaller logs. "
                        "Type 'done' on a new line when finished.")
    parser.add_argument('--sort', choices=['newest','oldest'], help="--sort newest|oldest [OPTIONAL ARGUMENTS]. Used for sorting all files found in json destinations.")
    parser.add_argument('--file', help="--file <filename> [OPTIONAL ARGUMENTS]. Used for processing a give file name.")
    #Exclusive for --batch
    parser.add_argument('--skip-confirm', action='store_true', help="--skip-confirm. Used with --batch only. Skips all file confirmation prompts.")
    parser.add_argument('--recent', type=int, help="--recent <int>. Used with --batch. Uses the n most recent files.")
    parser.add_argument('--old', type=int, help="--old <int>. Used with --batch. Uses the n oldest files.")
    # General flags
    parser.add_argument('--range', help="--range start:end. Only outputs points from start to end")
    parser.add_argument('--out', help="--out <name>. Allows overriding of the default file name.")
    parser.add_argument('--path-name', help="--path-name <name>. Allows overriding of the default JerryIO Path name")
    parser.add_argument('--config', help="--config <json_filename>. Allows changing of the config file without changing any text")
    parser.add_argument('--fast', action='store_true', help="--fast. Allows skipping of all questions. Defaults are used, and all files are accepted")
    parser.add_argument('--verbose', action='store_true', help="--verbose. Prints more information about the path created")
    parser.add_argument('--no-output', action='store_true', help="--no-output. The provided command will not produce an output file. Used for statistics")
    # Exclusive for --latest
    parser.add_argument('--index', type=int, help="--index <int>. Used with --latest. Used for getting a specific index of the latest files.")
    # For JerryIO Reverse parsing
    parser.add_argument('--cast', action='store_true', help="--cast --out <name> --file <filename> --config <json_filename> [OPTIONAL ARGUMENTS]"
                        "Convert a JerryIO LemLib V0.5 File into regular autonomous code based on a provided json template")


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
            # Load paths from JSON config (or DEFAULT_CONFIG_FILE)
            json_path = args.config or DEFAULT_CONFIG_FILE
            with open(json_path, "r") as f:
                cfg = json.load(f)

            scan_paths = cfg.get("scan_paths_latest", [])
            if not scan_paths:
                raise FileNotFoundError("No destinations in config: scan_paths_latest")

            # Collect all .txt files from the paths
            txt_files = []
            for p in scan_paths:
                if os.path.isdir(p):
                    # search for *.txt in this folder
                    for f_name in os.listdir(p):
                        if f_name.endswith(".txt"):
                            txt_files.append(os.path.join(p, f_name))
                elif os.path.isfile(p) and p.endswith(".txt"):
                    txt_files.append(p)
                else:
                    # invalid path, ignore or print warning
                    if args.verbose:
                        print(f"[WARN] Ignoring invalid path: {p}")

            if not txt_files:
                raise FileNotFoundError("No .txt files found in scan_paths_latest directories/files")

            # Sort files by modification time
            txt_files.sort(key=lambda f: os.path.getmtime(f), reverse=(args.sort != "oldest"))

            # Pick file
            if args.fast:
                input_file = txt_files[0]
                print(f"[FAST] Using latest file: {input_file}")
            else:
                # Ask user for confirmation
                for f in txt_files:
                    if confirm(f"Use this file for conversion: {f}?"):
                        input_file = f
                        break
                if input_file is None:
                    raise RuntimeError("No file selected for conversion with --latest")
                    sys.exit(1)
        else:
            raise ValueError("You must specify either --file <filename> or --latest with --cast")

        try:
            # Perform conversion
            cast_jerry_path(
                input_file=input_file,
                output_file=args.out,
                config_file=args.config or DEFAULT_CONFIG_FILE,
                verbose=args.verbose
            )
        except (ValueError, FileNotFoundError) as e:
            print(f"[ERROR] {e}")
            sys.exit(1)
        sys.exit(0)

    # ---------- VALIDATION ----------
    if not (args.batch or args.latest or args.file or args.paste or args.cast):
        parser.error("Must specify a command")

    if not args.batch and (args.skip_confirm or args.recent or args.old):
        parser.error("Batch-only flags used without --batch")

    if args.recent and args.old:
        parser.error("--recent and --old cannot be used together")

    params = CONFIG if args.fast else prompt_for_config(CONFIG)
    base_out = ensure_output_folder(params['output_folder'])

    # ---------- BATCH ----------
    if args.batch:
        try:
            logs = get_logs_from_paths(params['scan_paths_batch'])
            logs = sort_logs(logs, newest=not args.old)
            if args.recent:
                logs = logs[:args.recent]
            if args.old:
                logs = logs[:args.old]

            folder = args.out or args.path_name or os.path.splitext(os.path.basename(logs[0]))[0]
            batch_dir = os.path.join(base_out, folder)
            os.makedirs(batch_dir, exist_ok=True)

            for log in logs:
                if not args.skip_confirm and not confirm(f"Process {log}?"):
                    continue
                with open(log) as f:
                    base = os.path.splitext(os.path.basename(log))[0]
                    out = os.path.join(batch_dir, sanitize_batch_name(args.out, base))

                    run_processor(
                        f.readlines(),
                        base,
                        params,
                        out + "-jerryio.txt",
                        path_name=sanitize_batch_name(args.path_name, base),
                        range_str=args.range,
                        verbose=args.verbose,
                        no_output=args.no_output,
                        config_file=args.config or DEFAULT_CONFIG_FILE
                    )

        except (ValueError, FileNotFoundError, IOError) as e:
            print(f"[ERROR][batch] {e}")
            sys.exit(1)

    # ---------- LATEST ----------
    elif args.latest:
        try:
            logs = sort_logs(get_logs_from_paths(params['scan_paths_latest']), newest=True)
            if not logs:
                print("No .log files found in scan_paths_latest")
                return

            if args.index is not None:
                if args.index < 0 or args.index >= len(logs):
                    print(f"Error: --index {args.index} out of range (0-{len(logs)-1})")
                    return
                logs = [logs[args.index - 1]]

            log = logs[0]
            # Check file with user if not fast
            if not args.fast and not confirm(f"Use {log}?"):
                print("Aborted!")
                sys.exit(1)

            with open(log) as f:
                base = os.path.splitext(os.path.basename(log))[0]
                out = os.path.join(base_out, args.out or base)

                run_processor(
                    f.readlines(),
                    base,
                    params,
                    out + "-jerryio.txt",
                    path_name=sanitize_batch_name(args.path_name, base),
                    range_str=args.range,
                    verbose=args.verbose,
                    no_output=args.no_output,
                    config_file=args.config or DEFAULT_CONFIG_FILE
                )

        except (ValueError, FileNotFoundError, IOError) as e:
            print(f"[ERROR][latest] {e}")
            sys.exit(1)

    # ---------- FILE ----------
    elif args.file:
        try:
            with open(args.file) as f:
                base = os.path.splitext(os.path.basename(args.file))[0]
                out = os.path.join(base_out, args.out or base)

            if not args.fast and not confirm(f"Use {args.file}?"):
                print("Aborted!")
                sys.exit(1)

                run_processor(
                    f.readlines(),
                    base,
                    params,
                    out + "-jerryio.txt",
                    path_name=sanitize_batch_name(args.path_name, base),
                    range_str=args.range,
                    verbose=args.verbose,
                    no_output=args.no_output,
                    config_file=args.config or DEFAULT_CONFIG_FILE
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
                if line.lower() == 'done':
                    break
                lines.append(line)

            raw, table, json_obj, *_ = create_jerryio_json(
                apply_range(process_log_data(lines, params), args.range),
                args.path_name or "Pasted_Path",
                params
            )

            if not args.no_output:
                write_output(
                    os.path.join(base_out, "Pasted_Path-jerryio.txt"),
                    raw,
                    table,
                    json_obj
                )

        except (ValueError, IOError) as e:
            print(f"[ERROR][paste] {e}")
            sys.exit(1)


if __name__ == "__main__":
    main()