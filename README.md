# PRECISE Truck

The PRECISE Truck is an adaptation of the autonomous [RCCar](https://github.com/RISE-Dependable-Transport-Systems/RCCar) tailored for PRECISE project to run Security Chaos Engineering experiments and validation work.

## Installing Prerequisites (on Ubuntu 22.04/24.04) & Building

Install latest **MAVSDK 2.x** (support for version 3.x is not yet available).

- **Option 1:** Download pre-built package from [here](https://github.com/mavlink/MAVSDK/releases) and install using:

  ```bash
  sudo dpkg -i libmavsdk-dev*.deb
  ```

- **Option 2:** Build MAVSDK from source. Simple [scripts](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/tools/build_MAVSDK) can be found in the **WayWise** repository.

Install other dependencies:

```bash
sudo apt install git build-essential cmake libqt5serialport5-dev libgpiod-dev
```

Clone the repository in any directory (workspace) and build:

```bash
git clone git@github.com:RISE-Dependable-Transport-Systems/precise-truck.git precise-truck
cd precise-truck
git submodule update --init
cmake -S . -B build
make -C build -j
```

If the build is successful, two executables will be created directly in the `build/` directory:

- `build/PreciseTruck` — the autonomous truck application
- `build/ControlTower` — the ground control station UI

## Running the Truck

You can run PreciseTruck from the project root using:

```bash
./build/PreciseTruck
```

PreciseTruck has a range of parameters configurable using a JSON file:

| Parameter                         | Type     | Default                                       | Description                                                         |
| --------------------------------- | -------- | --------------------------------------------- | ------------------------------------------------------------------- |
| `truck_id`                        | `int`    | `1`                                           | Unique ID for the truck.                                            |
| `trailer_id`                      | `int`    | `25`                                          | Unique ID for the trailer.                                          |
| `attach_trailer`                  | `bool`   | `false`                                       | Whether to attach the trailer to the truck.                         |
| `truck_length`                    | `double` | `0.5`                                         | Length of the truck in meters.                                      |
| `truck_width`                     | `double` | `0.21`                                        | Width of the truck in meters.                                       |
| `trailer_length`                  | `double` | `0.96`                                        | Length of the trailer in meters.                                    |
| `trailer_width`                   | `double` | `0.21`                                        | Width of the trailer in meters.                                     |
| `trailer_wheelbase`               | `double` | `0.64`                                        | Distance between trailer axles.                                     |
| `axis_distance`                   | `double` | `0.3`                                         | Distance between the truck’s front and rear axles.                  |
| `turn_radius`                     | `double` | `0.67`                                        | Turning radius of the truck.                                        |
| `servo_center`                    | `double` | `0.5`                                         | Servo neutral position (normalized).                                |
| `servo_range`                     | `double` | `0.5`                                         | Servo motion range (normalized).                                    |
| `angle_sensor_offset`             | `double` | `90.0`                                        | Calibration offset for the steering angle sensor (degrees).         |
| `pure_pursuit_radius`             | `double` | `1.0`                                         | Default lookahead distance for the Pure Pursuit algorithm (meters). |
| `speed_to_rpm_factor`             | `double` | `5190`                                        | Conversion factor between target speed and motor RPM.               |
| `adaptive_radius`                 | `bool`   | `true`                                        | Enables adaptive Pure Pursuit radius.                               |
| `repeat_route`                    | `bool`   | `false`                                       | Whether the vehicle repeats the waypoint route automatically.       |
| `use_vesc_imu`                    | `bool`   | `true`                                        | Use VESC’s onboard IMU instead of external BNO055.                  |
| `update_period_ms`                | `int`    | `25`                                          | Period (in milliseconds) for vehicle state updates.                 |
| `control_tower_ip`                | `string` | `"127.0.0.1"`                                 | IP address of the MAVSDK Control Tower.                             |
| `control_tower_port`              | `int`    | `14540`                                       | UDP port for MAVSDK Control Tower connection.                       |
| `route_file_path`                 | `string` | `""`                                          | Path to RTCM correction server configuration file.                  |
| `speed_limit_regions_file_path`   | `string` | `""`                                          | Path to JSON file defining speed limit regions.                     |
| `enu_ref`                         | `string` | `"57.7171924432987, 12.962759215969157, 0.0"` | ENU reference coordinates: latitude, longitude, altitude.           |
| `route_file_path`                 | `string` | `""`                                          | Path to the waypoint route XML file.                                |
| `gnss_simulation_noise_sigma_pos` | `double` | `0.0`                                         | GNSS simulation positional noise standard deviation (meters).       |
| `gnss_simulation_noise_sigma_vel` | `double` | `0.0`                                         | GNSS simulation velocity noise standard deviation (m/s).            |
| `log_directory_path`              | `string` | `""`                                          | Directory where log files will be created.                          |

> **Note on file paths:**
> Paths specified in the JSON configuration (e.g., `rtcm_info_file_path`, `speed_limit_regions_file_path`, `route_file_path`, `log_directory_path`) can be either **absolute** or **relative**.
>
> - If relative, they are interpreted **relative to the project root** when running a binary compiled in a normal build environment.
> - If the binary is run in a Docker container or from another environment where the project root is not available, relative paths are interpreted **relative to the executable location**.

In addition to providing path to a JSON configuration file, command-line arguments can be used to override the truck and trailer IDs and flag to attach the trailer using the following options:

| Option                  | Description                                                    |
| ----------------------- | -------------------------------------------------------------- |
| `-c, --config <file>`   | Path to a JSON configuration file containing truck parameters. |
| `-i, --truck-id <id>`   | Set the truck ID (overrides JSON value).                       |
| `-l, --trailer-id <id>` | Set the trailer ID (overrides JSON value).                     |
| `-a, --attach-trailer`  | Flag to attach the trailer to the truck (overrides JSON ).     |

#### Example Usage of Command-line Arguments

From the project root directory:

```bash
# Use a JSON configuration file
./build/PreciseTruck --config config/truck1.json

# Override truck ID and attach trailer
./build/PreciseTruck -c config/truck1.json -i 2 -l 26 -a
```

## Running ControlTower

[ControlTower](https://github.com/RISE-Dependable-Transport-Systems/ControlTower) is a ground control station UI (built as part of this project) that can visualize and control the truck over MAVLink. It is built alongside PreciseTruck and its executable is at `build/ControlTower`.

Run it from the project root:

```bash
./build/ControlTower
```

Once running:

1. In ControlTower, go to **Connections** and add an UDP connection matching the truck's `control_tower_ip` and `control_tower_port` (default: `127.0.0.1:14540`).
2. The truck will appear on the map after connecting. You can visualize its position, upload waypoint routes, and monitor its state in real time.

> ControlTower and PreciseTruck can run on the same machine (using `127.0.0.1`) or on separate machines on the same network, as long as the IP/port settings match.

## 🐳 Docker Container

PRECISE Truck can be run inside a Docker container for convenient development and testing.

From the project root directory:

```bash
docker build -t precise-truck:latest .
```

Run PreciseTruck (from any directory) using a JSON configuration file:

```bash
docker run --rm --network host precise-truck:latest ./PreciseTruck --config config/truck1.json
```

### Resetting State for Chaos Experiments

The container is designed to be **stateless** - no persistent volumes are used by default. To reset the system state and start clean:

```bash
# Stop the running container
docker stop precise-truck

# Start a fresh instance
docker run --rm --name precise-truck --network host precise-truck:latest ./PreciseTruck --config config/truck1.json
```

Each container restart provides a completely clean environment, ideal for repeatable chaos engineering experiments.
