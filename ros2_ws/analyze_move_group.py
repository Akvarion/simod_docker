#!/usr/bin/env python3
"""
Scansiona la workspace e analizza TUTTI i <robot>_moveit_config
tenendo conto del package ROS 2 che li contiene.  Per ogni coppia
(package, robot) verifica quante istanze di move_group sono lanciate
e se mancano alcuni parametri critici.
"""

import os, re, xml.etree.ElementTree as ET
from collections import OrderedDict

# ──────────────────────────────────────────────────────────────────────────
# Helper 1 – trova la root del package (la cartella con package.xml)
# ──────────────────────────────────────────────────────────────────────────
def find_ros_package_root(start_path: str) -> str | None:
    cur = os.path.abspath(start_path)
    while cur != os.path.dirname(cur):          # finché non arrivo a /
        if os.path.exists(os.path.join(cur, "package.xml")):
            return cur
        cur = os.path.dirname(cur)
    return None


def get_package_name(pkg_root: str) -> str:
    pkg_xml = os.path.join(pkg_root, "package.xml")
    try:
        tree = ET.parse(pkg_xml)
        return tree.getroot().findtext("name").strip()
    except Exception:
        # fallback: usa il nome della dir
        return os.path.basename(pkg_root)


# ──────────────────────────────────────────────────────────────────────────
# Helper 2 – scopri tutti i *_moveit_config presenti
# ──────────────────────────────────────────────────────────────────────────
def discover_moveit_configs(workspace_root: str = ".") -> "dict[tuple,str]":
    """
    Ritorna OrderedDict { (pkg_name, robot_name) : full_path_to_moveit_config }
    """
    entries: dict[tuple, str] = {}
    for dirpath, dirnames, _ in os.walk(workspace_root):
        # print(f"Scansione {dirpath}...")
        for d in dirnames:
            if not d.endswith("_moveit_config"):
                continue
            robot = d[:-len("_moveit_config")]
            cfg_dir = os.path.join(dirpath, d)
            pkg_root = find_ros_package_root(cfg_dir)
            if pkg_root is None:        # fuori da un package ROS? salta
                continue
            pkg_name = get_package_name(pkg_root)
            entries[(pkg_name, robot)] = cfg_dir

    # ordinato per nome-package, poi robot
    return OrderedDict(sorted(entries.items()))


# ──────────────────────────────────────────────────────────────────────────
# Helper 3 – trova launch/*.py dentro a moveit_config
# ──────────────────────────────────────────────────────────────────────────
def find_launch_files(moveit_cfg_path: str) -> list[str]:
    launch_files = []
    for dirpath, _, filenames in os.walk(moveit_cfg_path):
        if os.path.basename(dirpath) != "launch":
            continue
        for fname in filenames:
            if fname.endswith(".py"):
                launch_files.append(os.path.join(dirpath, fname))
    return launch_files


# ──────────────────────────────────────────────────────────────────────────
# Helper 4 – estrai Node(move_group) + parametri
# ──────────────────────────────────────────────────────────────────────────
def parse_move_group_nodes(launch_files: list[str]) -> list[dict]:
    instances = []
    node_re = re.compile(
        r'Node\([^)]*package\s*=\s*"moveit_ros_move_group"[^)]*\)', re.S
    )
    for lf in launch_files:
        with open(lf, "r", encoding="utf-8") as f:
            txt = f.read()

        for m in node_re.finditer(txt):
            snippet = m.group(0)
            ns = re.search(r'namespace\s*=\s*"([^"]+)"', snippet)
            params = re.findall(r'parameters\s*=\s*\[([^\]]+)\]', snippet, re.S)
            instances.append(
                {
                    "file": lf,
                    "namespace": ns.group(1) if ns else "",
                    "params_raw": params,
                }
            )
    return instances


def param_missing(param_list: list[str], key: str) -> bool:
    return key not in ",".join(param_list)


# ──────────────────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────────────────
def main(workspace_root="."):
    entries = discover_moveit_configs(workspace_root)
    if not entries:
        print("❌ Nessuna cartella *_moveit_config trovata.")
        return

    for (pkg, robot), cfg_path in entries.items():
        print(f"\n=== Package: {pkg} | Robot: {robot} ===")

        launches = find_launch_files(cfg_path)
        if not launches:
            print("  ✖ Nessun launch/*.py dentro il moveit_config.")
            continue

        nodes = parse_move_group_nodes(launches)
        n = len(nodes)
        if n == 0:
            print("  ✖ Nessun Node(move_group) trovato nei launch.")
            continue
        elif n == 1:
            print("  ✔ Una sola istanza di move_group.")
        else:
            print(f"  ⚠ {n} istanze di move_group (MoveIt 2 è single-robot di default).")

        # verifica parametri essenziali
        issues = []
        for inst in nodes:
            if param_missing(inst["params_raw"], "trajectory_execution/allowed_start_tolerance"):
                issues.append(f"    − manca 'trajectory_execution/allowed_start_tolerance' in {inst['file']}")
            if param_missing(inst["params_raw"], "use_sim_time"):
                issues.append(f"    − manca 'use_sim_time' in {inst['file']}")

        if issues:
            print("  Problemi parametri:")
            for iss in issues:
                print(iss)
        else:
            print("  Nessun problema di parametri rilevato.")


if __name__ == "__main__":
    # punta qui alla root della tua workspace (es. './src' se usi overlay)
    main(workspace_root=".")
