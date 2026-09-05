# DynamicTerrainSystem

Map imagery and elevation terrain streaming for Gazebo Harmonic.

The plugin downloads terrain tiles around a moving aircraft and adds them to the server's sensor scene. As the aircraft moves, the terrain follows it, nearby imagery gains detail, and off-screen meshes and textures are released. Collision terrain is built separately around the vehicle.

Features:

- satellite imagery and custom tile sources
- Terrarium elevation data, with optional flat terrain
- progressive texture refinement and reusable terrain pages
- camera-based eviction of off-screen rendering resources
- bounded in-memory image and elevation caches
- local download cache for revisiting an area
- moving collision heightmap and temporary startup ground

Built for Gazebo Harmonic (`gz-sim8`) with Ogre2. One model is tracked per world. PX4 and a separate camera streaming plugin are optional.

## Requirements

Tested on Ubuntu 24.04. Install Gazebo Harmonic using the [official Ubuntu instructions](https://gazebosim.org/docs/harmonic/install_ubuntu/), then install the build dependencies:

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake pkg-config \
    libgz-sim8-dev libgz-plugin2-dev libgz-rendering8-ogre2-dev \
    libgz-gui8-dev libogre-next-2.3-dev \
    libcurl4-openssl-dev libopencv-dev \
    libprotobuf-dev protobuf-compiler \
    qtbase5-dev qtdeclarative5-dev
```

An active camera sensor and the Ogre2 Sensors system are needed to render terrain. Internet access is needed for tiles that are not already cached.

## Build

```bash
git clone https://github.com/syvixi/gz-dynamic-terrain-plugin.git
cd gz-dynamic-terrain-plugin

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

The system library is `build/libgz-dynamic-terrain-system.so`. Keep `libgz-dynamic-terrain-core.so` alongside it.

Add the build directory to Gazebo's plugin path:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

Launch Gazebo or PX4 from the same terminal. You can increase `-j2` if you have enough memory for more parallel compiler jobs.

## Server setup

For PX4, open `<PX4_PATH>/src/modules/simulation/gz_bridge/server.config`. Add the terrain system inside `<plugins>`, after the existing Ogre2 Sensors system:

```xml
<plugin entity_name="*" entity_type="world"
        filename="gz-sim-sensors-system"
        name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>

<plugin entity_name="*" entity_type="world"
        filename="libgz-dynamic-terrain-system.so"
        name="custom::DynamicTerrainSystem"/>
```

Keep the other PX4 systems and load each system only once. The [example server configuration](examples/server.config) shows the placement; do not replace your configuration if it contains other plugins you need.

For a standalone Gazebo world, add the same two plugin entries directly inside `<world>`, without the `entity_name` and `entity_type` attributes. Keep the world's other systems, including Physics.

## World setup

Set the geographic origin inside your world's `<world>` element:

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>37.4319</latitude_deg>
  <longitude_deg>-122.1697</longitude_deg>
  <elevation>30</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

Replace the latitude, longitude, and elevation with your starting location. The plugin uses Gazebo's geographic transform, including the world heading, to position terrain.

By default, `align_origin_to_ground` shifts the downloaded elevation so the ground at the origin is at local Z = 0. Set it to `false` to keep the source elevation relative to the world's elevation reference. Remove or reposition any existing ground plane that would overlap the generated terrain.

## Model setup

Copy the block from [examples/plugin_snippet.sdf](examples/plugin_snippet.sdf) into the `<model>` that terrain should follow, not into a link or camera sensor. For PX4's Cessna, this is usually `<PX4_PATH>/Tools/simulation/gz/models/rc_cessna/model.sdf`.

Set `camera_names` to the camera sensor names in your model:

```xml
<camera_names>camera_front,camera_down</camera_names>
```

Only list cameras that exist. The renderer waits until all listed cameras are available before evicting off-screen pages. Omit this setting to consider all cameras in the server scene.

The example uses a 7.5 km terrain radius and concentrates detailed imagery around the terrain patch below the aircraft. Despite its name, `bottom_camera_only` selects a ground-distance region, not the exact footprint or direction of a camera.

For video streaming, the separate [GstPlaneCameraSystem plugin](https://github.com/syvixi/px4-gazebo-gstreamer-camera-plugin) can be used with the same cameras. It is not required by the terrain plugin.

## Start the simulation

After configuring the server, world, and model, start your PX4 target:

```bash
cd /path/to/PX4-Autopilot
make px4_sitl gz_rc_cessna
```

Or start your configured standalone world:

```bash
gz sim -r /path/to/your_world.sdf
```

The first load takes longer while imagery and elevation tiles download. View the terrain through a camera sensor. With `diagnostics` enabled, messages prefixed with `[DynamicTerrain]` report downloads, terrain updates, and resource usage.

## Configuration

These settings go inside the model's `custom::DynamicTerrainConfig` block. Values below are defaults; the example adjusts a few of them for aircraft use.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `visual_radius_m` | `7500` | Radius of the visual terrain area, in metres. |
| `visual_geometry_zoom` | `14` | Tile zoom used to divide the terrain mesh into pages. |
| `visual_elevation_zoom` | `13` | Elevation source zoom for the visual mesh. |
| `visual_mesh_cells_per_tile` | `64` | Mesh subdivisions per tile edge. |
| `visual_page_texture_max_size` | `2048` | Maximum texture edge length, in pixels. |
| `visual_page_cache_mb` | `128` | Refined-image cache budget in RAM, in MiB. |
| `decoded_dem_cache_mb` | `256` | Decoded elevation cache budget in RAM, in MiB. |
| `visual_frustum_eviction` | `true` | Release rendering resources for off-screen pages. |
| `visual_offscreen_frames` | `30` | Off-screen grace period, in render frames. |
| `download_concurrency` | `4` | Maximum concurrent tile downloads. |
| `download_per_host` | `1` | Maximum concurrent downloads to one host. |
| `enable_collision` | `true` | Generate the moving collision heightmap. |
| `align_origin_to_ground` | `true` | Align terrain at the origin to local Z = 0. |
| `cache_dir` | `~/.cache/gz_dynamic_terrain` | Location of cached downloads and generated terrain files. |

For lower VRAM use, start by reducing `visual_page_texture_max_size` to `1024` or reducing the visual radius. Lowering texture resolution also limits the imagery detail available on each page. A shorter off-screen grace period frees resources sooner but may cause more uploads when the camera turns.

The two RAM cache budgets are not a limit on total Gazebo memory. Active terrain, pending updates, cameras, physics, and the renderer need additional memory. The disk cache is separate and has no automatic size limit.

Collision resolution changes with the model's local Z coordinate when `dynamic_zoom` is enabled; it is not based on height above the terrain. For fixed resolution, set `dynamic_zoom` to `false` and choose `static_zoom`.

### Tile sources

The example uses `google_satellite` imagery and `terrarium` elevation. For your own imagery service, use an XYZ URL:

```xml
<imagery_provider>custom</imagery_provider>
<imagery_url>https://your-tile-server.example/{z}/{x}/{y}.jpg</imagery_url>
<imagery_extension>jpg</imagery_extension>
```

Replace the example URL with your provider's endpoint. Set `elevation_provider` to `flat` if you do not need elevation downloads. Tile availability, access permissions, attribution, and usage limits depend on the provider; the project license does not cover third-party map data. Keep provider credentials out of published SDF files.

## Tests and measured results

Run the terrain regression tests after building:

```bash
ctest --test-dir build --output-on-failure \
    -R '^(terrain_types|terrain_builder_mapping|ogre2_resource_lifecycle|example_plugin_filenames)$'
```

All four passed on September 5, 2026. The tests use synthetic data without downloading map tiles. Coverage includes:

- 56 coordinate round trips across two geographic origins and four headings, at horizontal distances up to 100 km, with an error tolerance below 1 mm. This checks coordinate conversion, not the absolute accuracy of map data.
- Texture coordinates and collision alignment, plus five image-cache configurations exercised over 40 pages each. The 128 MiB cache case holds at most ten 2048 × 2048 RGB images (120 MiB of pixel data); replaced and evicted images must be released.
- Ogre2 resource cleanup over 20 terrain generations, eviction and reload during 20 camera turns, and 39 checked mountain-scene frames across terrain transitions.
- Plugin library names in the SDF and server examples.

Measured with a Release build on Ubuntu 24.04, Intel Core i7-13700HX, GCC 13.3, Gazebo Sim 8.11.0, Rendering 8.2.3, and OpenCV 4.9.0:

| Test executable | Median wall time, 5 runs | Highest peak RSS, 5 runs |
| --- | ---: | ---: |
| `dynamic-terrain-types-test` | 0.02 s | 43.9 MiB |
| `dynamic-terrain-builder-test` | 3.18 s | 233.5 MiB |
| `dynamic-terrain-ogre2-lifecycle-test` | 0.62 s | 548.1 MiB |

These are short regression-test measurements, not flight FPS, GPU benchmarks, or proof of long-running memory stability. RSS includes the entire test process and its libraries. The renderer test uses software OpenGL; CTest reports it as skipped if a rendering engine or scene cannot be created.

To repeat a measurement, run the following five times, changing the executable name for each test:

```bash
LIBGL_ALWAYS_SOFTWARE=1 /usr/bin/time -f 'wall=%e s peak_RSS=%M KiB' \
    ./build/dynamic-terrain-builder-test
```

## Troubleshooting

- **Plugin not found:** check that `GZ_SIM_SYSTEM_PLUGIN_PATH` points to the build directory in the terminal that launches Gazebo or PX4. Keep both terrain libraries together.
- **No terrain:** check the world coordinates, the model configuration block, the Ogre2 Sensors system, and that a camera is active. Look for tile download errors in the server output.
- **Terrain missing at altitude:** check the camera's far clipping distance as well as the visual radius. Increasing either can increase rendering work.
- **Off-screen memory does not drop:** check camera names first. Cache reuse and allocator behaviour can keep process RSS above the amount of live terrain data, so inspect resource diagnostics as well as system memory.

## Repository files

[.gitignore](.gitignore) excludes build directories, CMake output, compiled libraries, test logs, editor files, and Python bytecode. It also excludes a repository-local `.cache/` directory; the default terrain cache lives outside the repository. Source files, examples, test fixtures, documentation images, and [LICENSE](LICENSE) remain trackable. Ignore rules do not remove files that Git already tracks, and they are not a substitute for checking files for credentials before publishing.

## License

BSD 3-Clause. Copyright (c) 2026 Alex Chazov. See [LICENSE](LICENSE).
