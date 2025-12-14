# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.


## Commit Messages

High-level, 2-3 lines max:
```
<What you did in one line>

<Why/how in 1-2 short sentences>
```

Example:
```
Refactor UR5e to match IIWA textbook pattern

Use LoadScenario + MakeHardwareStation like IIWA examples.
Simplify driver from 100 to 45 lines.
```

No bullet lists. No file-by-file details. Just the essence.


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


## UR5e Code Patterns

**Simulation** - Use manipulation library's `MakeHardwareStation` with `!InverseDynamicsDriver`:
```python
from manipulation.station import LoadScenario, MakeHardwareStation
from ur5e_description.ur5e_driver import UR5E_PACKAGE_XML

scenario = LoadScenario(data="""
directives:
- add_directives:
    file: package://ur5e_description/ur5e_netft_probe.dmd.yaml
model_drivers:
    ur5e: !InverseDynamicsDriver {}
""")
station = MakeHardwareStation(scenario, meshcat=meshcat, package_xmls=[UR5E_PACKAGE_XML])
```

**Hardware** - Use `UR5eDriver` directly:
```python
from ur5e_description.ur5e_driver import UR5eDriver
driver = builder.AddSystem(UR5eDriver(robot_ip="192.168.1.102"))
# Ports match simulation: ur5e.desired_state, ur5e.state_estimated
```