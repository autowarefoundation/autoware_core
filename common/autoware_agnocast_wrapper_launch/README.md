# autoware_agnocast_wrapper_launch

Launch frontend extension for Autoware nodes that can be run in more than one way.

## Why

An Autoware node can be started in three ways, and which one is right depends on how it
communicates:

- as a process of its own, on plain rclcpp;
- loaded into a component container, where intra-process communication avoids a copy;
- as a process of its own, on Agnocast, where skipping DDS discovery cuts the startup time and
  the CPU a container would otherwise cost.

A node on Agnocast is a process of its own, which is where Agnocast pays off. The other pairing
exists — `agnocast_env.launch.xml` swaps the container for one from `agnocast_components` when
`ENABLE_AGNOCAST=1` — but the action below does not reach for it, and never puts a node on Agnocast
into a container. So a node that uses Agnocast has to be launched one way with `ENABLE_AGNOCAST=1`
and another way without it, and writing both forms out by hand means every launch site carries two
copies of the same parameters and remappings, which drift apart.

## `<autoware_node>`

`<autoware_node>` is written like `<node>`, with `target` saying where the node ends up.

```xml
<autoware_node
  pkg="autoware_pointcloud_preprocessor"
  exec="random_downsample_filter_node"
  name="random_downsample_filter"
  target="$(var pointcloud_container_name)"
>
  <param from="$(var random_downsample_filter_param_path)"/>
  <remap from="input" to="voxel_grid_downsample/pointcloud"/>
  <remap from="output" to="$(var output/pointcloud)"/>
  <extra_arg name="use_intra_process_comms" value="$(var use_intra_process)"/>
</autoware_node>
```

`<autoware_node>` is parsed by the same code the built-in `<node>` and `<composable_node>` use, so
it takes everything they do, it is written once, and parameter values keep their YAML type.

### Attributes

| Attribute   | Default  | Description                                                           |
| ----------- | -------- | --------------------------------------------------------------------- |
| `pkg`       | required | Package holding the executable and the component                      |
| `exec`      | required | Executable name                                                       |
| `name`      | —        | Node name, the same in every mode                                     |
| `namespace` | —        | Node namespace; omit to keep the namespace the action is evaluated in |
| `mode`      | `auto`   | `auto` or `rclcpp`                                                    |
| `target`    | —        | Container to load into; omit or leave empty for a process of its own  |

Everything else `<node>` takes is taken too, and applies to the standalone form: `args`,
`ros_args`, `output`, `exec_name`, `respawn`, `respawn_delay`, `launch-prefix`, `cwd`, `on_exit`,
`shell`, `emulate_tty`, and `if` / `unless`.

### Child elements

| Element       | Applies to         | Description                                              |
| ------------- | ------------------ | -------------------------------------------------------- |
| `<param>`     | both forms         | Parameter, or a file of them via `from`                  |
| `<remap>`     | both forms         | Remap rule                                               |
| `<extra_arg>` | the container form | `rclcpp::NodeOptions` argument, e.g. intra-process comms |
| `<env>`       | the standalone one | Environment variable for the process                     |

### What applies to which form

A component has no process of its own, so nothing said about a process reaches it: `<env>`,
`output`, `args`, `respawn` and the rest are used when the node is launched on its own and ignored
when it is loaded into a container, which has its own environment and its own stdout.
`<extra_arg>` is the mirror image — it is the container's `rclcpp::NodeOptions`,
`use_intra_process_comms` above all, and a standalone node makes its own.

So a node is written once, in full, and each form takes the part that means something to it. A
launch site converted from a `<node>`/`<composable_node>` pair keeps both halves.

`<env>` is merged with the environment the action sets itself, and `ENABLE_AGNOCAST` and
`LD_PRELOAD` are the action's to decide: use `mode="rclcpp"` to keep a node off Agnocast, not an
`<env>` of your own.

### Modes

`target` decides the form, as it would anywhere else: given, the node is loaded into that
container; omitted, it is a process of its own. A `target` that evaluates to the empty string counts
as omitted, so one variable can decide the form for a whole launch file:

```xml
<autoware_node pkg="..." exec="..." name="..." target="$(var pointcloud_container_name)"/>
```

`mode` only says whether to use Agnocast, and there is nothing to decide unless the node was built
with Agnocast support and the launch runs with `ENABLE_AGNOCAST=1`:

| `mode`   | Behaviour                                      |
| -------- | ---------------------------------------------- |
| `auto`   | the default; Agnocast wherever it is available |
| `rclcpp` | rclcpp, even where Agnocast is available       |

Anywhere else — a workspace built without Agnocast, or a launch without `ENABLE_AGNOCAST=1` —
`mode` changes nothing and the node is launched exactly as written. There is no value that asks for
Agnocast and fails when it is unavailable: a launch file states the node it wants, and the build
and the run decide what that node can be.

A node running on Agnocast gets a process of its own and `target` is ignored, with a line in the
log saying so.

`mode="rclcpp"` keeps one node off Agnocast in a workspace that otherwise runs on it, as a fallback
while debugging. `agnocast_env.launch.xml` has a `use_agnocast` argument for the same purpose, and
stays in use either way, since resolving a container to its Agnocast counterpart is its job and not
this action's. `<autoware_node>` does not read it, taking `ENABLE_AGNOCAST` from the environment and
`mode` from the launch file, so say it with `mode` here.

`mode="rclcpp"` without `target` is simply a process of its own on rclcpp and nothing else has
to change. With `target` it goes back into the container, but a component takes the environment of
the container it is loaded into and launch cannot set it per component, so that container has to be
started with `ENABLE_AGNOCAST=0`:

```xml
<node_container pkg="rclcpp_components" exec="component_container" name="pointcloud_container" namespace="">
  <env name="ENABLE_AGNOCAST" value="0"/>
</node_container>
```

Without it the container exits the moment the component is loaded: Agnocast checks its own
`LD_PRELOAD` on startup and calls `exit()` when the heaphook is not there, taking the container and
everything else in it down. The action can only warn, which it does whenever it loads a node into a
container while `ENABLE_AGNOCAST=1`.

The container also has to be one put there for the purpose. At `ENABLE_AGNOCAST=1` a launch tree's
own containers come from `agnocast_env.launch.xml`, which resolves them to
`agnocast_component_container` with the heaphook preloaded, so loading into one of those would give
the node Agnocast after all rather than the rclcpp `mode="rclcpp"` asked for.

### The heaphook

A node running on Agnocast is started with `libagnocast_heaphook.so` in its `LD_PRELOAD`. The action
takes the path from the `agnocast_heaphook_path` launch configuration, the same one
`agnocast_env.launch.xml` declares, and falls back to
`/opt/ros/$ROS_DISTRO/lib/libagnocast_heaphook.so`:

```xml
<let name="agnocast_heaphook_path" value="/path/to/libagnocast_heaphook.so"/>
```

The heaphook has to be the one built against the `agnocastlib` the workspace was built against; a
mismatched pair fails at startup. The action checks the file is there before it launches anything,
so a wrong path is reported by launch rather than by the node's loader.

Only the node's own process is given the preload. If the launch itself was started with a heaphook
in `LD_PRELOAD`, that copy is replaced rather than added to, since a second copy of the heaphook in
one process does not work.

### Resolving the component

`autoware_agnocast_wrapper_register_node()` registers an `autoware_node_plugins` resource named
`<package>__<executable>`, holding `<component class>;<transport>`, so `<autoware_node>` only needs
the executable. The class is the one to load into a container. The transport is `agnocast` or
`rclcpp` depending on the `ENABLE_AGNOCAST` the package was **built** with, which is what makes
`mode` a question only a build with Agnocast support can answer.

The resource is why `target` needs a node registered that way. Launching a process of its own takes
only the executable and works for any node, but loading one into a container takes the component
class, and the standard `rclcpp_components` index lists a package's classes without saying which
executable goes with which — there is nothing to look the class up by. So `<autoware_node>` with a
`target` reports an error for a node registered with plain `rclcpp_components_register_node()`
rather than guessing at the class.

That is the intended scope: writing a node out in both forms is a cost Agnocast imposes, and no
rclcpp-only launch site needs this action today. Should one turn up, the action can grow an
explicit `plugin` attribute for it then.
