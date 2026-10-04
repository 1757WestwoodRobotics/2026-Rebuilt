#!/usr/bin/env python3
"""
LogTracer Flamegraph Viewer Tool
--------------------------------
Parses WPILib .wpilog files, extracts `LogTracer/*` timing data and `Timing/RobotPeriodic/*` entries,
builds hierarchical timing call-trees, and outputs an interactive HTML Flamegraph visualization.

Usage:
  uv run python tools/log_flamegraph_viewer.py path/to/logfile.wpilog [-o output.html] [--open]
"""

import argparse
import html
import json
import os
import sys
import webbrowser
from typing import Dict, Any, List
from wpiutil.log import DataLogReader


def parse_wpilog(log_filepath: str) -> Dict[str, Dict[str, Any]]:
    """Reads wpilog file and aggregates average, max, min, and count for LogTracer topics."""
    reader = DataLogReader(log_filepath)
    if not reader.isValid():
        print(f"Error: Invalid or unreadable WPILOG file: {log_filepath}")
        sys.exit(1)

    # Entry ID -> metadata map
    entries: Dict[int, Dict[str, str]] = {}
    # Topic Name -> list of float values (in ms)
    topic_data: Dict[str, List[float]] = {}

    for record in reader:
        if record.isControl():
            if record.isStart():
                data = record.getStartData()
                entries[data.entry] = {
                    "name": data.name,
                    "type": data.type,
                }
        else:
            entry_id = record.getEntry()
            if entry_id not in entries:
                continue
            entry_info = entries[entry_id]
            name = entry_info["name"]

            # Filter for LogTracer and Timing topics (including RealOutputs/ and ReplayOutputs/ prefixes)
            if not ("LogTracer" in name or "Timing" in name):
                continue


            # Read double / float records
            try:
                if entry_info["type"] in ("double", "float"):
                    val_ms = record.getDouble()
                elif entry_info["type"] == "double[]":
                    vals = record.getDoubleArray()
                    val_ms = vals[0] if len(vals) > 0 else 0.0
                else:
                    continue

                if name not in topic_data:
                    topic_data[name] = []
                topic_data[name].append(val_ms)
            except (ValueError, TypeError, AttributeError, IndexError) as _:
                continue


    if not topic_data:
        print("Warning: No `LogTracer/*` or `Timing/*` records found in log file.")

    stats: Dict[str, Dict[str, Any]] = {}
    for name, vals in topic_data.items():
        if not vals:
            continue
        avg_ms = sum(vals) / len(vals)
        max_ms = max(vals)
        min_ms = min(vals)
        stats[name] = {
            "avg_ms": round(avg_ms, 4),
            "max_ms": round(max_ms, 4),
            "min_ms": round(min_ms, 4),
            "count": len(vals),
        }

    return stats


def build_tree_from_stats(stats: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Converts flat LogTracer and Timing keys into a clean hierarchical call tree."""
    # Find total robot periodic time if recorded
    total_robot_periodic = 20.0
    for key in stats:
        if key.endswith("RobotPeriodic/TotalMS"):
            total_robot_periodic = stats[key]["avg_ms"]
            break

    root: Dict[str, Any] = {
        "name": "RobotPeriodic",
        "value": total_robot_periodic,
        "avg_ms": total_robot_periodic,
        "max_ms": total_robot_periodic,
        "min_ms": total_robot_periodic,
        "count": stats.get("Timing/RobotPeriodic/TotalMS", {}).get("count", 0),
        "children": [],
    }

    nodes: Dict[str, Dict[str, Any]] = {"": root}

    # Sort topics by number of path components
    sorted_items = sorted(stats.items(), key=lambda item: item[0].count("/"))

    for topic_name, data in sorted_items:
        clean_path = topic_name
        for prefix in ("RealOutputs/", "ReplayOutputs/"):
            if clean_path.startswith(prefix):
                clean_path = clean_path[len(prefix) :]

        if clean_path.endswith("/TotalMS"):
            clean_path = clean_path[:-len("/TotalMS")]

        # Place top-level categories (LogTracer, Timing) under RobotPeriodic root
        if clean_path.startswith("Timing/RobotPeriodic/TotalMS"):
            continue  # Root already represents TotalMS

        parts = [p for p in clean_path.split("/") if p]
        if not parts:
            continue

        curr_path = ""
        parent_node = root

        for i, part in enumerate(parts):
            curr_path = f"{curr_path}/{part}" if curr_path else part
            if curr_path not in nodes:
                is_leaf = i == len(parts) - 1
                node_avg = data["avg_ms"] if is_leaf else 0.0
                node_max = data["max_ms"] if is_leaf else 0.0
                node_min = data["min_ms"] if is_leaf else 0.0
                node_cnt = data["count"] if is_leaf else 0

                new_node: Dict[str, Any] = {
                    "name": part,
                    "value": node_avg,
                    "avg_ms": node_avg,
                    "max_ms": node_max,
                    "min_ms": node_min,
                    "count": node_cnt,
                    "children": [],
                }

                nodes[curr_path] = new_node
                parent_node["children"].append(new_node)

            parent_node = nodes[curr_path]
            if i == len(parts) - 1:
                parent_node["value"] = data["avg_ms"]
                parent_node["avg_ms"] = data["avg_ms"]
                parent_node["max_ms"] = data["max_ms"]
                parent_node["min_ms"] = data["min_ms"]
                parent_node["count"] = data["count"]

    # Post-process: compute inclusive timing for non-leaf parent nodes from their children
    def compute_inclusive_timing(node: Dict[str, Any]) -> float:
        if node["children"]:
            child_sums = sum(compute_inclusive_timing(child) for child in node["children"])
            if node["avg_ms"] == 0.0 or node["avg_ms"] < child_sums:
                node["avg_ms"] = child_sums
                node["value"] = child_sums
        return node["avg_ms"]

    compute_inclusive_timing(root)
    return root



def generate_html_flamegraph(
    stats: Dict[str, Dict[str, Any]], log_filename: str
) -> str:
    """Generates an interactive HTML Flamegraph string using D3.js flamegraph visualization."""
    tree_data = build_tree_from_stats(stats)
    tree_json = json.dumps(tree_data, indent=2)

    html_template = f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>LogTracer Flamegraph - {html.escape(os.path.basename(log_filename))}</title>
  <style>
    body {{
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      background-color: #0b1320;
      color: #e2e8f0;
      margin: 0;
      padding: 24px;
    }}
    .header {{
      display: flex;
      justify-content: space-between;
      align-items: center;
      border-bottom: 1px solid #1e293b;
      padding-bottom: 16px;
      margin-bottom: 24px;
    }}
    h1 {{
      font-size: 1.4rem;
      font-weight: 600;
      margin: 0;
      color: #60a5fa;
      letter-spacing: -0.01em;
    }}
    .file-tag {{
      background: #111c2e;
      padding: 6px 12px;
      border-radius: 0px;
      font-family: monospace;
      border: 1px solid #1e2e4a;
      color: #94a3b8;
    }}
    .card {{
      background-color: #111c2e;
      border: 1px solid #1e2e4a;
      border-radius: 0px;
      padding: 20px;
      box-shadow: none;
    }}
    #flamegraph-container {{
      position: relative;
      width: 100%;
      min-height: 450px;
    }}

    .node-bar {{
      box-sizing: border-box;
      border: 1px solid #0b1320;
      border-radius: 0px;
      position: absolute;
      cursor: pointer;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      padding: 4px 8px;
      font-size: 12px;
      font-weight: 500;
      color: #e2e8f0;
      transition: filter 0.1s ease;
    }}
    .node-bar:hover {{
      filter: brightness(1.25);
      z-index: 10;
    }}
    #tooltip {{
      position: absolute;
      display: none;
      background: #090e17;
      border: 1px solid #2563eb;
      border-radius: 0px;
      padding: 12px;
      color: #f8fafc;
      font-size: 13px;
      pointer-events: none;
      z-index: 100;
    }}
    .tooltip-title {{
      font-weight: 600;
      color: #60a5fa;
      margin-bottom: 4px;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      margin-top: 24px;
      font-size: 14px;
    }}
    th, td {{
      padding: 10px 14px;
      text-align: left;
      border-bottom: 1px solid #1e2e4a;
    }}
    th {{
      background: #0b1320;
      color: #64748b;
      text-transform: uppercase;
      font-size: 11px;
      letter-spacing: 0.05em;
    }}
    tr:hover td {{
      background: #17253d;
    }}

  </style>
</head>
<body>
  <div class="header">
    <h1>🔥 LogTracer Execution Flamegraph</h1>
    <div class="file-tag">{html.escape(os.path.basename(log_filename))}</div>
  </div>

  <div class="card">
    <div id="flamegraph-container"></div>
  </div>

  <div id="tooltip"></div>

  <div class="card" style="margin-top: 24px;">
    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 16px;">
      <h2 style="font-size: 1.1rem; color: #60a5fa; margin: 0; font-weight: 600;">Timing Breakdowns Table</h2>
      <div style="display: flex; gap: 8px;">
        <button id="btn-tree-view" style="background: #2563eb; color: #ffffff; border: 1px solid #1d4ed8; padding: 6px 14px; border-radius: 0px; font-weight: 500; cursor: pointer;">Hierarchy Tree View</button>
        <button id="btn-flat-view" style="background: #1e2e4a; color: #94a3b8; border: 1px solid #1e2e4a; padding: 6px 14px; border-radius: 0px; font-weight: 500; cursor: pointer;">Flat Topic View</button>
      </div>
    </div>
    <table>
      <thead>
        <tr>
          <th id="th-name" style="cursor: pointer;">Topic / Node Name ↕</th>
          <th id="th-avg" style="cursor: pointer; text-align: right;">Avg Time (ms) ↕</th>
          <th id="th-max" style="cursor: pointer; text-align: right;">Max Time (ms) ↕</th>
          <th id="th-min" style="cursor: pointer; text-align: right;">Min Time (ms) ↕</th>
          <th id="th-count" style="cursor: pointer; text-align: right;">Sample Count ↕</th>
        </tr>
      </thead>
      <tbody id="table-body"></tbody>
    </table>
  </div>

  <script>
    const treeData = {tree_json};
    const statsData = {json.dumps(stats, indent=2)};

    const colors = [
      "#1e3a8a", "#1e40af", "#1d4ed8", "#2563eb", "#3b82f6",
      "#172554", "#1e293b", "#334155", "#0284c7", "#0369a1"
    ];

    function getColor(name, depth) {{
      let hash = 0;
      for (let i = 0; i < name.length; i++) {{
        hash = name.charCodeAt(i) + ((hash << 5) - hash);
      }}
      const index = Math.abs(hash) % colors.length;
      return colors[index];
    }}


    function renderFlamegraph() {{
      const container = document.getElementById("flamegraph-container");
      container.innerHTML = "";
      const width = container.clientWidth;
      const rowHeight = 32;

      // Collect nodes by depth level
      const levels = [];

      function traverse(node, depth, xStart, xWidth) {{
        if (!levels[depth]) levels[depth] = [];
        levels[depth].push({{
          name: node.name,
          avg_ms: node.avg_ms,
          max_ms: node.max_ms,
          min_ms: node.min_ms,
          count: node.count,
          xStart: xStart,
          xWidth: xWidth
        }});

        if (node.children && node.children.length > 0) {{
          const totalChildValue = node.children.reduce((acc, c) => acc + (c.avg_ms || 0.1), 0);
          let currX = xStart;
          for (const child of node.children) {{
            const childWidth = (totalChildValue > 0) ? (xWidth * ((child.avg_ms || 0.1) / totalChildValue)) : (xWidth / node.children.length);
            traverse(child, depth + 1, currX, childWidth);
            currX += childWidth;
          }}
        }}
      }}

      traverse(treeData, 0, 0, width);

      const tooltip = document.getElementById("tooltip");

      levels.forEach((nodesAtDepth, depth) => {{
        nodesAtDepth.forEach(node => {{
          if (node.xWidth < 2) return; // skip tiny bars
          const bar = document.createElement("div");
          bar.className = "node-bar";
          bar.style.top = (depth * (rowHeight + 4)) + "px";
          bar.style.left = node.xStart + "px";
          bar.style.width = (node.xWidth - 2) + "px";
          bar.style.height = rowHeight + "px";
          bar.style.backgroundColor = getColor(node.name, depth);
          bar.textContent = `${{node.name}} (${{node.avg_ms.toFixed(2)}}ms)`;

          bar.addEventListener("mousemove", (e) => {{
            tooltip.style.display = "block";
            tooltip.style.left = (e.pageX + 15) + "px";
            tooltip.style.top = (e.pageY + 15) + "px";
            tooltip.innerHTML = `
              <div class="tooltip-title">${{node.name}}</div>
              <div><b>Average:</b> ${{node.avg_ms.toFixed(3)}} ms</div>
              <div><b>Max:</b> ${{node.max_ms.toFixed(3)}} ms</div>
              <div><b>Min:</b> ${{node.min_ms.toFixed(3)}} ms</div>
              <div><b>Samples:</b> ${{node.count}}</div>
            `;
          }});

          bar.addEventListener("mouseleave", () => {{
            tooltip.style.display = "none";
          }});

          container.appendChild(bar);
        }});
      }});

      container.style.height = (levels.length * (rowHeight + 4)) + "px";
    }}

    let currentView = "tree"; // "tree" or "flat"
    let sortColumn = "avg_ms";
    let sortAsc = false;

    function renderTable() {{
      const tbody = document.getElementById("table-body");
      tbody.innerHTML = "";

      if (currentView === "flat") {{
        let items = Object.entries(statsData).map(([topic, data]) => ({{
          name: topic,
          avg_ms: data.avg_ms,
          max_ms: data.max_ms,
          min_ms: data.min_ms,
          count: data.count,
        }}));

        items.sort((a, b) => {{
          let valA = a[sortColumn];
          let valB = b[sortColumn];
          if (typeof valA === "string") return sortAsc ? valA.localeCompare(valB) : valB.localeCompare(valA);
          return sortAsc ? valA - valB : valB - valA;
        }});

        items.forEach(item => {{
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td style="font-family: monospace;">${{item.name}}</td>
            <td style="color: #38bdf8; font-weight: bold; text-align: right;">${{item.avg_ms.toFixed(3)}} ms</td>
            <td style="color: #ef4444; text-align: right;">${{item.max_ms.toFixed(3)}} ms</td>
            <td style="color: #4ade80; text-align: right;">${{item.min_ms.toFixed(3)}} ms</td>
            <td style="text-align: right;">${{item.count}}</td>
          `;
          tbody.appendChild(tr);
        }});
      }} else {{
        // Tree View Rendering
        function flattenTree(node, depth = 0, prefix = "") {{
          let result = [];
          const indent = "&nbsp;&nbsp;&nbsp;&nbsp;".repeat(depth);
          const icon = node.children && node.children.length > 0 ? "📂 " : "📄 ";
          result.push({{
            name: `${{indent}}${{icon}}${{node.name}}`,
            clean_name: node.name,
            avg_ms: node.avg_ms,
            max_ms: node.max_ms,
            min_ms: node.min_ms,
            count: node.count,
            depth: depth,
            children: node.children || []
          }});

          if (node.children) {{
            let childrenCopy = [...node.children];
            childrenCopy.sort((a, b) => {{
              let valA = a[sortColumn];
              let valB = b[sortColumn];
              if (typeof valA === "string") return sortAsc ? valA.localeCompare(valB) : valB.localeCompare(valA);
              return sortAsc ? valA - valB : valB - valA;
            }});

            childrenCopy.forEach(child => {{
              result = result.concat(flattenTree(child, depth + 1, prefix));
            }});
          }}
          return result;
        }}

        const treeItems = flattenTree(treeData);
        treeItems.forEach(item => {{
          const tr = document.createElement("tr");
          tr.innerHTML = `
            <td style="font-family: monospace;">${{item.name}}</td>
            <td style="color: #38bdf8; font-weight: bold; text-align: right;">${{item.avg_ms.toFixed(3)}} ms</td>
            <td style="color: #ef4444; text-align: right;">${{item.max_ms.toFixed(3)}} ms</td>
            <td style="color: #4ade80; text-align: right;">${{item.min_ms.toFixed(3)}} ms</td>
            <td style="text-align: right;">${{item.count}}</td>
          `;
          tbody.appendChild(tr);
        }});
      }}
    }}

    // Setup Header Event Listeners
    const sortHeaders = [
      {{ id: "th-name", col: "name" }},
      {{ id: "th-avg", col: "avg_ms" }},
      {{ id: "th-max", col: "max_ms" }},
      {{ id: "th-min", col: "min_ms" }},
      {{ id: "th-count", col: "count" }},
    ];

    sortHeaders.forEach(hdr => {{
      document.getElementById(hdr.id).addEventListener("click", () => {{
        if (sortColumn === hdr.col) {{
          sortAsc = !sortAsc;
        }} else {{
          sortColumn = hdr.col;
          sortAsc = false;
        }}
        renderTable();
      }});
    }});

    // Setup View Mode Toggle Event Listeners
    document.getElementById("btn-tree-view").addEventListener("click", () => {{
      currentView = "tree";
      document.getElementById("btn-tree-view").style.background = "#2563eb";
      document.getElementById("btn-tree-view").style.color = "#ffffff";
      document.getElementById("btn-tree-view").style.border = "1px solid #1d4ed8";
      document.getElementById("btn-flat-view").style.background = "#1e2e4a";
      document.getElementById("btn-flat-view").style.color = "#94a3b8";
      document.getElementById("btn-flat-view").style.border = "1px solid #1e2e4a";
      renderTable();
    }});

    document.getElementById("btn-flat-view").addEventListener("click", () => {{
      currentView = "flat";
      document.getElementById("btn-flat-view").style.background = "#2563eb";
      document.getElementById("btn-flat-view").style.color = "#ffffff";
      document.getElementById("btn-flat-view").style.border = "1px solid #1d4ed8";
      document.getElementById("btn-tree-view").style.background = "#1e2e4a";
      document.getElementById("btn-tree-view").style.color = "#94a3b8";
      document.getElementById("btn-tree-view").style.border = "1px solid #1e2e4a";
      renderTable();
    }});


    window.addEventListener("resize", renderFlamegraph);
    renderFlamegraph();
    renderTable();
  </script>
</body>
</html>
"""
    return html_template



def main():
    parser = argparse.ArgumentParser(
        description="Extract LogTracer timing data from .wpilog file and generate an interactive flamegraph HTML report."
    )
    parser.add_argument("wpilog", help="Path to the .wpilog file")
    parser.add_argument(
        "-o",
        "--output",
        default="flamegraph_report.html",
        help="Output HTML file path (default: flamegraph_report.html)",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Automatically open generated HTML flamegraph report in web browser",
    )

    args = parser.parse_args()

    if not os.path.exists(args.wpilog):
        print(f"Error: File not found: {args.wpilog}")
        sys.exit(1)

    print(f"Parsing log file: {args.wpilog}...")
    stats = parse_wpilog(args.wpilog)
    html_content = generate_html_flamegraph(stats, args.wpilog)

    with open(args.output, "w", encoding="utf-8") as f:
        f.write(html_content)

    print(f"✅ Flamegraph successfully generated: {os.path.abspath(args.output)}")

    if args.open:
        webbrowser.open(f"file://{os.path.abspath(args.output)}")


if __name__ == "__main__":
    main()
