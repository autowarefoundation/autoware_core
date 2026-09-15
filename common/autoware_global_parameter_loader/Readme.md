# Autoware Global Parameter Loader

This package is to set common ROS parameters to each node.

## Usage

Add the following lines to the launch file of the node in which you want to get global parameters.

```xml
<!-- Global parameters -->
  <include file="$(find-pkg-share autoware_global_parameter_loader)/launch/global_params.launch.py">
    <arg name="vehicle_model" value="$(var vehicle_model)"/>
  </include>
```

The vehicle model parameter is read from `config/vehicle_info.param.yaml` in `vehicle_model`\_description package.

### Loading additional parameter files

Any parameter file written in the `/**: ros__parameters:` format can be loaded as global parameters
by listing it in `global_parameter_loader_param_files`, separated by commas.

```xml
<!-- Global parameters -->
  <include file="$(find-pkg-share autoware_global_parameter_loader)/launch/global_params.launch.py">
    <arg name="vehicle_model" value="$(var vehicle_model)"/>
    <arg name="global_parameter_loader_param_files" value="$(find-pkg-share $(var vehicle_model)_description)/config/my_params.param.yaml"/>
  </include>
```

Files that do not exist are skipped without an error, so optional parameter files can be listed
unconditionally. When the same parameter appears in multiple files, the value of the last file wins.

## Assumptions / Known limits

- vehicle_info is always loaded by this launcher; any other parameter file has to be given through `global_parameter_loader_param_files`.
- Paths in `global_parameter_loader_param_files` must not contain a comma, and `~` and environment variables are not expanded. Use `$(find-pkg-share ...)` or `$(env ...)` instead.
- A listed file that exists but cannot be read, or is not in the `/**: ros__parameters:` format, aborts the launch with an error message containing the file path.
