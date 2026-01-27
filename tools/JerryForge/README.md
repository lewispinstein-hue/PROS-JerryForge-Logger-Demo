# JerryForge Smart Path Plotter CLI - v1.0   

The JerryPlotter CLI casts raw robot log files into JerryIO-compatible path data, and vice-versa. It handles coordinate normalization, path thinning, and coordinate offsets.

Stop guessing where your robot drove. JerryPlotter turns your raw terminal logs into editable, visual paths in seconds, allowing you to record manual driving and cast it into autonomous code.

**Optional helper (Linux/MacOS):**  
By running:  

> alias pickAName='python3 "/full/path/to/jerryforge.py" "$@"'  

You can create an alias and run JerryForge through that alias. Example:  

> pickAName --latest --fast. 

**Tip:** You can make this permanent by editing your `~/.bashrc`, `~/.zshrc`, or `~/.bash_aliases`. 

---

## HOW TO RUN

**General Syntax:**  
`python3 jerryforge.py [COMMAND] [OPTIONAL FLAGS]`

**Example:**  
`python3 jerryforge.py --latest --fast`

---

# Overview:  
  
JerryForge has two parts:  
> 1. PathBuilder: This turns robot logs into a viewable & editable file. (Most commands)
> 2. PathCaster: This turns a JerryIO Path into robot code (`--cast`)

**Requirements:**

## A Quick Start

Log your robot run:

```shell
pros terminal | tee "run.log"
```

***Important:** You must set up something that logs your robot’s pose in this specific format:*  
`[DATA],x,y,theta,right_vel,left_vel`

This creates a file and dumps the terminal output into it.

Drive around a little bit, but don't be too crazy.

Cast it to JerryIO:

```shell
python3 jerryforge.py --latest --fast
```

Open the output file in https://path.jerryio.com

You now have a fully editable JerryIO path that recreates the route you just drove — ready to be tuned, refined, and turned into autonomous code.

---

## Commands and Flags Overview

> Commands decide **what** data is processed.  
> Flags decide **how** that command behaves.

### Commands

| Command | Description |
|--------|-------------|
| `--latest` | Automatically selects the most recent `.log` file in the configured directories. |
| `--batch` | Processes multiple `.log` files at once (entire directory). |
| `--file <path>` | Processes a specific file chosen by the user. |
| `--paste` | Processes data pasted directly into the terminal (type `done` to finish). |
| `--cast` | **PathCaster**: Casts a JerryIO file (LemLib V0.5) into code using a template. |
| `--sort <type>` | Sorts files by `oldest` or `newest` (valid with `--latest` or `--batch`). |

### Flags

| Flag | Description |
|------|-------------|
| `--fast` | Automation mode. Skips confirmation prompts and uses JSON defaults. |
| `--out <name>` | Overrides the default output filename and/or location. |
| `--config <file>` | Uses a specific JSON configuration file instead of the default. |
| `--verbose` | Prints detailed analytics (efficiency, fidelity loss, bounds). |
| `--path-name <name>` | Sets the internal JerryIO path name (batch default: `name_filename`). |
| `--range A:B` | Filters points by index. Example: `0:3000` processes only the first 3000 points. |
| `--index <N>` | **Latest only**: Selects the Nth file in the list. |
| `--new <N>` | **Batch only**: Processes the N most recent files. |
| `--old <N>` | **Batch only**: Processes the N oldest files. |
| `--skip-confirm` | **Batch only**: Skips per-file confirmation prompts. |

---

## COMMANDS (Deep-Dive)

### `--latest`

Scans the configured directories for the most recently modified `.log` file.

- Standard: Finds the file and asks for confirmation (y/n)
- With `--fast`: Automatically selects and processes the file

---

### `--file <filename> or <filepath>`

Processes a specific file chosen by the user. Can be in any folder.

- Standard: Prompts you to type the file name/path
- Validation: Will exit if the file is not found

---

### `--batch`

Processes **every** `.log` file in the current directory.

- Standard: Lists all found files and asks for confirmation
- With `--fast`: Skips the list and casts all files immediately using default settings
- `--out <filename>` can be used with `--batch`

Output naming:

- With `--out`:  
  `<filename>-<original_filename>-jerryio.log`
- Without `--out`:  
  `<original_filename>-jerryio.log`

This command always dumps output into a new folder determined by:

1. The name provided by `--out <filename>`
2. The path name provided by `--path-name <path_name>`
3. The name of the first file processed (fallback)

If `output_folder` is set, the resulting file will always end up inside of it.

---

### `--paste`

Processes data pasted directly into the terminal window.

Best for smaller logs, as there is no need to drag a folder or copy a path. This also gives control over what data is passed into JerryIO.

- How to: Paste your logs, then type `done` on a new line

---

### `--sort oldest | newest`

Sorts all files by date modified and splits them into:

- Files in `scan_paths_latest`
- Files in `scan_paths_batch`

Useful for finding the exact index of a file or determining what `--latest` can see.

---

## FLAGS (Deep-Dive)

All flags can be used concurrently.

Example:

```shell
python3 jerryforge.py --latest --fast --verbose --out my_route
```

### `--fast`

The automation flag. When added to any command:

1. Skips confirmation prompts for file selection
2. Skips manual configuration prompts (density, tolerance, offsets, etc.)
3. Uses values stored in `jerry_plotter-defaults.json` or the file provided by `--config`

Example:

```shell
python3 jerryforge.py --latest --fast
```

---

### `--verbose`

Prints detailed analytics such as path length and coordinate bounds.

Example:

```shell
python3 jerryforge.py --latest --verbose
```

---

### `--config <file>`

Allows you to use a config file other than the default `jerry_plotter-defaults.json`.

Example:

```shell
python3 jerryforge.py --latest --config skills_config.json --fast
```

---

### `--range start:end`

Filters a specific range of points.

- Measurement unit: points
- Can be used with `--batch`

Example:

```shell
python3 jerryforge.py --latest --range 500:2500
```

This writes only points 500–2499 into the JerryIO file.

This flag can also extract the first *N* seconds of movement.

Assuming:
- Logging at 100 Hz
- Density = 1

Example:

```shell
python3 jerryforge.py --latest --range 0:300
```

This extracts ~3 seconds of movement.

---

### `--out <filename>`

Overrides the default output filename and/or location.

Example:

```shell
python3 jerryforge.py --latest --out skills_route
```

Result: `skills_route-jerryio.txt`

---

### `--path-name`

Sets the name of the path as it appears in JerryIO.

For `--batch` runs:

```
<path_name>_<original_file_name>
```

---

### `--new`

Limits batch processing to the N most recent `.log` files.

Only valid with `--batch`.

Example:

```shell
python3 jerryforge.py --batch --new 5
```

---

### `--old`

Limits batch processing to the N oldest `.log` files.

Only valid with `--batch`.

Example:

```shell
python3 jerryforge.py --batch --old 3
```

---

### `--skip-confirm`

Skips per-file confirmation during batch processing.

Only valid with `--batch`.

Example:

```shell
python3 jerryforge.py --batch --new 2 --skip-confirm
```

---

### `--index`

Selects a specific index for `--latest`.

Only valid with `--latest`.

Example:

```shell
python3 jerryforge.py --latest --index 2 --fast
```

---

## Using JerryIO PathCaster

**Syntax:**

```shell
python3 jerryforge.py --cast (--file <filename> | --latest) --out <outputName>
```

This casts a JerryIO file (LemLib V0.5 format) into code based on a template.

- Works with `--verbose` and `--config`
- Config must include:
  - `render_invisible_waypoints` (boolean)
  - `template` (string)

### Template Variables

1. `${x}`: X-Position 
2. `${y}`: Y-Position
3. `${theta}`: Heading
4. `${speed}`: Movement speed
5. `${distance}`: Distance from previous waypoint
6. `${iteration}`: Current waypoint

This works by going through all waypoints (excludes invisible based on `render_invisible_waypoints` value) and creating a matching output based on the template.

---

## Using JerryForge PathBuilder

**Syntax:**

```shell
python3 jerryforge.py (--latest | --file <filename> | --batch | --paste) [FLAGS]
```

PathBuilder casts raw robot log data into a JerryIO-compatible path file.
It parses pose logs, applies thinning and normalization, and outputs an editable path that can be visualized and refined in path.jerryio.com.  

This is the primary workflow for turning manual driving into a structured autonomous path.  

* Works with all standard flags (`--fast`, `--verbose`, `--config`, `--out`, `--range`, etc.)
* Requires robot logs to contain pose data in the format:  
`[DATA],x,y,theta,right_vel,left_vel`

### **What PathBuilder Does**
1. Scans for input data based on the selected command:  
`--latest`: Most recent log file in configured paths  
`--file`: A specific log file  
`--batch`: Multiple log files  
`--paste`: Manually pasted log data  
2. Extracts valid `[DATA]` lines from the input.  
3. Applies preprocessing and thinning:  
`Density filtering`  
`XY and heading tolerances`  
`Offset adjustments`  
`Visibility rules` for waypoints  
4. Generates a JerryIO-compatible path file:
- Includes both visible and invisible waypoints  
- Preserves smooth motion while keeping the editor usable  
- Writes the output to a `.txt` file ready to be opened in:  
[Jerryio Website](https://path.jerryio.com)

## Common Q & A

### Q: “JerryPlotter says no valid data was found”

**A:** Your log does not contain lines in the required format:

```
[DATA],x,y,theta,right_vel,left_vel
```

**Fix:** Ensure your robot logger prints exactly this tag and order.

Example:

```
[12.34] [INFO] [DATA],42.1,18.7,269.6,2134
```

Everything before `[DATA]` is ignored by the parser.

---

### Q: “`--latest` can’t find my log file”

**A:** JerryPlotter only scans directories listed in `scan_paths_latest`.

**Fix:** Add the correct directory to your config:

```json
"scan_paths_latest": [
  "./Logs",
  "/Users/yourname/RobotLogs"
]
```

This separation from `scan_paths_batch` is intentional.

---

### Q: “The path is shifted or rotated compared to the field”

**A:** Your odometry origin does not match JerryIO’s coordinate frame.

**Fix:** Adjust:

```json
"offset_x":
"offset_y":
"offset_theta":
```

---

### Q: “`--batch` didn’t process the files I expected”

**A:** `--batch` only scans `scan_paths_batch`.

**Fix:** Ensure your directory is listed or use `--new` / `--old`.

---

### Q: “Why are there so many invisible waypoints?”

**A:** Invisible waypoints preserve smooth motion; visible ones provide manual control.

Control this using `vis_interval`.

---

### Q: “Can I use this without path.jerryio.com?”

**A:** Yes.

Using `--cast`, you can generate robot commands directly from templates.