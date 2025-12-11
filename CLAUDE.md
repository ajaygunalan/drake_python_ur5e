# CLAUDE.md

Drake robotics manipulation examples with UR5e. See README.md for installation.

## Drake Documentation

Use **Ref MCP** for Drake/pydrake API lookups (e.g., `pydrake.visualization.ModelVisualizer`).

## Ref MCP vs Perplexity MCP: When to Use Which

### Quick Decision Tree
```
Need docs for a specific library/API? → Ref
Need current events/news? → Perplexity
Need official API reference? → Ref
Need synthesis from multiple sources? → Perplexity
Error with cryptic message? → Ref first, then Perplexity if stuck
Need cited sources? → Perplexity
```

### What Each Solves

| Ref MCP | Perplexity MCP |
|---------|----------------|
| Official documentation lookup | Real-time web search |
| API reference/method signatures | Current events & news |
| Library-specific patterns | Multi-source synthesis |
| Package changelogs | Cited research reports |
| Framework guides | Broad topic exploration |

### Use Ref When:
- You know *which* library, need *how* to use it
- Looking up exact method signatures, parameters
- Reading official docs, changelogs, migration guides
- Error points to a specific package/API

### Use Perplexity When:
- You don't know *which* library to use
- Comparing approaches across ecosystems
- Need information newer than docs (recent releases, issues)
- Want synthesized analysis with citations
- Stuck in a loop, need fresh perspective

### Combined Workflow Example
```
1. "Best library for force control in ROS2?" → Perplexity (exploration)
2. "ros2_control impedance controller API" → Ref (specific docs)
3. "Why is my controller not loading?" → Ref first (official troubleshooting)
4. Still stuck? → Perplexity (community solutions, GitHub issues)
```

### Cost/Speed Tradeoff
- **Ref**: Free, fast, authoritative but narrow scope
- **Perplexity**: API costs, slower, broad but may include noise

TL;DR: Ref = "I know what library, show me the docs." Perplexity = "I need to explore, compare, or find current info."
