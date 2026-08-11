"""Render parameterized SDF worlds and compose per-robot spawn actions.

Worlds are authored as xacro so that quantities like the ramp angle stay real
parameters instead of hand-computed vertex coordinates. Gazebo needs a concrete
``.sdf`` file, so the xacro is rendered at launch time into a generated directory and
that path is handed to Gazebo. Changing the ramp angle is therefore a launch argument,
not a rebuild and not an edit to a mesh.
"""

import os
import tempfile

import xacro


def generated_dir():
    """Return the directory that rendered worlds are written to.

    Uses the workspace-local ``generated/`` directory when one is available so the
    rendered SDF can be inspected after a run, falling back to a temporary directory.
    """
    workspace = os.environ.get("AMR_GENERATED_DIR")
    if workspace:
        os.makedirs(workspace, exist_ok=True)
        return workspace
    path = os.path.join(tempfile.gettempdir(), "amr_fleet_nav_generated")
    os.makedirs(path, exist_ok=True)
    return path


def render_world(xacro_path, mappings=None, out_name=None):
    """Render a world xacro to a concrete SDF file.

    Args:
        xacro_path: Path to the ``*.sdf.xacro`` source.
        mappings: Optional dict of xacro arguments; values are stringified.
        out_name: Optional output filename. Defaults to the source name with the
            ``.xacro`` suffix removed.

    Returns:
        Absolute path to the rendered ``.sdf``.
    """
    mappings = {key: str(value) for key, value in (mappings or {}).items()}
    doc = xacro.process_file(xacro_path, mappings=mappings)

    if out_name is None:
        out_name = os.path.basename(xacro_path)
        if out_name.endswith(".xacro"):
            out_name = out_name[: -len(".xacro")]

    out_path = os.path.join(generated_dir(), out_name)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(doc.toprettyxml(indent="  "))
    return out_path


def world_name_of(sdf_path):
    """Return the ``<world name=...>`` declared in a rendered SDF.

    Gazebo transport topics are scoped by world name, so the bridge and any ground-truth
    subscriber need this rather than assuming it matches the filename.
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(sdf_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"{sdf_path} contains no <world> element")
    return world.get("name")
