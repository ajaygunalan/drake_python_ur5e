# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.


## Context Awareness
Your context window will be automatically compacted as it approaches its limit, allowing you to continue working indefinitely from where you left off. Therefore, do not stop tasks early due to token budget concerns. As you approach your token budget limit, save your current progress and state to memory before the context window refreshes. Always be as persistent and autonomous as possible and complete tasks fully, even if the end of your budget is approaching. Never artificially stop any task early regardless of the context remaining.


## MCP Tools (Documentation & Code Search)

**4 Tools, 2 MCPs:**

| Tool | Purpose | Use When |
|------|---------|----------|
| `web_search_exa` | Web search → URLs | Finding official docs, broad discovery |
| `get_code_context_exa` | Code snippets | "How do I...?" questions, API patterns |
| `ref_search_documentation` | GitHub/private docs | Searching repos, indexed private docs |
| `ref_read_url` | Read URL → markdown | **Always use this** to read URLs (not WebFetch) |

**Decision Flow:**
1. Need working code? → `get_code_context_exa` first
2. Need official docs? → `web_search_exa` to find URL
3. Found a URL? → `ref_read_url` to read it
4. Searching GitHub specifically? → `ref_search_documentation`

**Skip searches for:** generic libraries (numpy, matplotlib, pandas, IPython), self-documenting errors, simple syntax, or anything answerable from training. Save MCP calls for domain-specific queries (pydrake, manipulation, ur_rtde).

**Example - Drake Jacobian question:**
```python
# 1. Find code pattern
get_code_context_exa("pydrake Jacobian pseudoinverse velocity control")
# → Returns: vq = pinv(J).dot(V_desired)

# 2. Find official docs
web_search_exa("pydrake CalcJacobianSpatialVelocity")
# → Returns: https://drake.mit.edu/pydrake/...

# 3. Read the docs
ref_read_url("https://drake.mit.edu/pydrake/pydrake.multibody.plant.html")
```

**Tips:**
- `get_code_context_exa`: Adjust `tokensNum` (1000-50000) based on complexity
- `web_search_exa`: Use `livecrawl: "preferred"` for fresh content
- `ref_search_documentation`: Add `ref_src=private` for indexed private repos (see below)
- Always pass EXACT URLs (including #hash) to `ref_read_url`


## Private Repos (Ref Indexed)

Domain-specific repos indexed for fast doc access:

| Repo | Purpose | Search Query |
|------|---------|--------------|
| `ajaygunalan-external-library/drake` | Robotics simulation | `drake pydrake ref_src=private` |
| `ajaygunalan-external-library/manipulation` | MIT manipulation | `manipulation ref_src=private` |
| `ajayexternlib/ur_rtde` | UR5e hardware control | `ur_rtde speedJ ref_src=private` |

**Adding New Dependencies:**
When discovering a domain-specific library via Exa, ask user: "Found [library]. Add to Ref as private repo for faster access?"


## UR5e Station

`ur5e_description/ur5e_station.py` - Hardware abstraction for sim + real UR5e:

```python
from ur5e_description.ur5e_station import MakeUR5eHardwareStation, GetPlantFromStation

station = MakeUR5eHardwareStation(meshcat=meshcat, control_mode="velocity")
plant = GetPlantFromStation(station)  # For Jacobian calculations
```

- **Ports**: `ur5e.velocity` (input), `ur5e.position_measured`, `ur5e.velocity_measured` (outputs)
- **Sim**: Uses `JointStiffnessController` (Kp=0, Kd=100) for velocity tracking with gravity comp
- **Hardware**: Uses `ur_rtde.speedJ()` - requires `pip install ur-rtde`
- **Home pose**: `[0, -90°, +90°, -90°, -90°, 0°]` defined in `ur5e_netft_probe.dmd.yaml`