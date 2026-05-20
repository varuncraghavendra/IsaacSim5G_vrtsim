# Isaac Sim Script Editor snippet -- enable + diagnose the Nova Carter
# front_hawk camera OmniGraph.
#
# How to use:
#   1. In Isaac Sim, open Window -> Script Editor.
#   2. Paste this entire file into a new tab.
#   3. Hit Run (Ctrl+Enter).
#   4. Read the printed report in the Script Editor's output pane.
#
# What it does:
#   - Enumerates every OmniGraph in the current stage and prints its
#     enabled/disabled state.
#   - Force-enables the graph at /World/Nova_Carter_ROS/front_hawk so its
#     IsaacROS2CameraHelper nodes start emitting frames.
#   - Prints every "ROS2*" node inside that graph along with the topicName
#     attribute it's set to publish on -- so you know exactly which ROS 2
#     topic to expect on the wire.
#
# After running, in a host shell:
#     source /opt/ros/humble/setup.bash
#     ros2 topic list | grep -iE "image|hawk"
# should show new topics; the dashboard will pick them up on its next 2 s scan.

import omni.usd
import omni.graph.core as og


TARGET_GRAPH = "/World/Nova_Carter_ROS/front_hawk"


def header(text: str) -> None:
    print()
    print("=" * 70)
    print(text)
    print("=" * 70)


def enumerate_graphs() -> list:
    """Return [(path, graph)] for every OmniGraph in the current stage."""
    stage = omni.usd.get_context().get_stage()
    out = []
    for prim in stage.TraverseAll():
        tname = prim.GetTypeName()
        if tname in ("OmniGraph", "ComputeGraph", "Graph"):
            path = str(prim.GetPath())
            try:
                graph = og.get_graph_by_path(path)
            except Exception:
                graph = None
            out.append((path, graph))
    return out


def is_enabled(graph) -> bool:
    """OmniGraph API renamed methods between Isaac versions; try both."""
    if graph is None:
        return False
    try:
        return not graph.is_disabled()
    except AttributeError:
        pass
    try:
        return bool(graph.get_enable_value())
    except AttributeError:
        pass
    try:
        return bool(graph.is_enabled())
    except AttributeError:
        pass
    return False


def force_enable(graph) -> str:
    """Best-effort enable; returns a status string."""
    if graph is None:
        return "graph object is None"
    for attempt in ("set_disabled", "set_enabled"):
        fn = getattr(graph, attempt, None)
        if fn is None:
            continue
        try:
            fn(False if attempt == "set_disabled" else True)
            return f"called {attempt}({'False' if attempt == 'set_disabled' else 'True'})"
        except Exception as exc:
            return f"{attempt} raised {exc!r}"
    return "no enable/disable method found on graph"


def dump_ros2_nodes(graph) -> None:
    """Print every ROS2-named node in the graph plus its topicName / enabled
    attributes. Helpful to discover the exact topic Isaac will publish on."""
    if graph is None:
        print("  (cannot inspect: graph is None)")
        return
    try:
        nodes = graph.get_nodes()
    except Exception as exc:
        print(f"  (cannot list nodes: {exc})")
        return
    for n in nodes:
        try:
            name = n.get_prim_path()
        except Exception:
            name = str(n)
        try:
            type_name = n.get_type_name()
        except Exception:
            type_name = "?"
        if "ROS2" not in type_name and "ros2" not in str(name).lower():
            continue
        print(f"  node: {name}    type: {type_name}")
        # Show topicName + enabled-ish attributes when present.
        for attr_name in ("inputs:topicName", "inputs:enabled",
                          "inputs:frameId", "inputs:queueSize",
                          "inputs:cameraPrim", "inputs:renderProductPath"):
            try:
                a = n.get_attribute(attr_name)
                v = a.get()
                print(f"      {attr_name} = {v!r}")
            except Exception:
                pass


# ---------------------------------------------------------------------------
header("All OmniGraphs in current stage")
all_graphs = enumerate_graphs()
if not all_graphs:
    print("  (no OmniGraphs found -- is a scene loaded?)")
for path, graph in all_graphs:
    enabled = is_enabled(graph)
    flag = "ENABLED" if enabled else "DISABLED"
    print(f"  {flag:>9}   {path}")

# ---------------------------------------------------------------------------
header(f"Force-enabling {TARGET_GRAPH}")
target_graph = None
for path, graph in all_graphs:
    if path == TARGET_GRAPH:
        target_graph = graph
        break

if target_graph is None:
    print(f"  graph not found at {TARGET_GRAPH}")
    print(f"  (it should appear in the list above -- check the prim path)")
else:
    before = is_enabled(target_graph)
    status = force_enable(target_graph)
    after = is_enabled(target_graph)
    print(f"  before: {'ENABLED' if before else 'DISABLED'}")
    print(f"  action: {status}")
    print(f"  after : {'ENABLED' if after else 'DISABLED'}")

# ---------------------------------------------------------------------------
header(f"ROS2 publisher nodes inside {TARGET_GRAPH}")
dump_ros2_nodes(target_graph)

# ---------------------------------------------------------------------------
header("Next steps")
print("  1. Make sure Isaac Sim is in PLAY mode (top toolbar play button).")
print("  2. From a host shell:")
print("       source /opt/ros/humble/setup.bash")
print("       ros2 topic list | grep -iE 'image|hawk'")
print("     The topicName(s) printed above should appear there.")
print("  3. The dashboard at http://localhost:8080 re-scans every 2 s and")
print("     will pick up new image topics automatically.")
print()
