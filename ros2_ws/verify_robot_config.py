#!/usr/bin/env python3
"""
Analizza tutte le coppie  <robot>_description  /  <robot>_moveit_config
presenti nella workspace e segnala i joint che compaiono nell’URDF/XACRO
ma non nei file MoveIt 2 corrispondenti, distinguendo i casi omonimi
tramite il nome del package ROS 2 che li contiene.
"""

import os
import re
import yaml
import xml.etree.ElementTree as ET
from collections import defaultdict, OrderedDict

# ───────────────────────────────────────────────
# Helper 0 – package.xml → nome del package
# ───────────────────────────────────────────────
def find_ros_package_root(start_path: str) -> str | None:
    cur = os.path.abspath(start_path)
    while cur != os.path.dirname(cur):
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
        return os.path.basename(pkg_root)


# ───────────────────────────────────────────────
# Utility: giunti & prefissi da URDF/XACRO
# ───────────────────────────────────────────────
def extract_joints_and_prefixes(urdf_xacro_files):
    joints, prefixes = set(), set()
    for p in urdf_xacro_files:
        with open(p, "r", encoding="utf-8") as f:
            txt = f.read()
        joints.update(re.findall(r'<joint[^>]*name="([^"]+)"', txt))
        prefixes.update(re.findall(r'(\w+)_\w+', txt))
    return joints, prefixes

def find_valid_moveit_cfg(path, max_up=2):
    """
    Ritorna il path che contiene `config/`.
    Se quello passato non la possiede, risale di livello finché la trova
    (o finché non supera `max_up`).  Restituisce None se non trovata.
    """
    cur = os.path.abspath(path)
    for _ in range(max_up + 1):
        if os.path.isdir(os.path.join(cur, "config")):
            return cur
        cur = os.path.dirname(cur)
    return None

def find_urdf_xacro_files(package_path, fallback_moveit_cfg=None):
    files = []
    for dp, _, fns in os.walk(package_path):
        for fn in fns:
            if fn.endswith((".urdf", ".xacro")):
                files.append(os.path.join(dp, fn))
    # se non abbiamo trovato nulla e ci hanno passato un fallback
    if not files and fallback_moveit_cfg:
        config_fallback = os.path.join(fallback_moveit_cfg, "config")
        #check existence of the config directory
        if not os.path.exists(config_fallback):
            print(f"[WARN] Config directory not found: {config_fallback}")
            return files
        cfg_urdfs = [
            os.path.join(fallback_moveit_cfg, "config", f)
            for f in os.listdir(os.path.join(fallback_moveit_cfg, "config"))
            if f.endswith((".urdf", ".xacro"))
        ]
        files.extend(cfg_urdfs)
    return files


def collect_joints_from_moveit_config(cfg_path: str) -> set[str]:
    """
    Raccoglie i nomi dei joint dai tre file MoveIt:
      • joint_limits.yaml
      • kinematics.yaml
      • moveit_controllers.yaml   (lista joints annidata)
    Restituisce un set con l’unione di tutti.
    """
    files = (
        "joint_limits.yaml",
        "kinematics.yaml",
        "moveit_controllers.yaml",
    )

    joints: set[str] = set()

    for fname in files:
        full = os.path.join(cfg_path, "config", fname)
        if not os.path.exists(full):
            continue

        with open(full, "r", encoding="utf-8") as fh:
            try:
                data = yaml.safe_load(fh) or {}
            except yaml.YAMLError:
                print(f"[WARN] parse fallito: {full}")
                continue

        # ── joint_limits / kinematics ───────────────────────────
        if fname != "moveit_controllers.yaml":
            if isinstance(data, dict):
                joints.update(data.keys())
            continue

        # ── moveit_controllers.yaml ─────────────────────────────
        # struttura attesa:
        # controller_list:
        #   - name: my_controller
        #     joints: [joint_a, joint_b, ...]
        if isinstance(data, dict) and "controller_list" in data:
            for ctrl in data["controller_list"] or []:
                if isinstance(ctrl, dict) and "joints" in ctrl:
                    joints.update(ctrl["joints"])

    return joints



# ───────────────────────────────────────────────
# 1) Scansione workspace  →  {(pkg, robot): paths}
# ───────────────────────────────────────────────
def discover_robot_packages(root="."):
    """
    Ritorna OrderedDict {
        (package_name, robot_name): {'description': path, 'moveit_config': path}
    }
    """
    robots = defaultdict(lambda: {"description": None, "moveit_config": None})

    for dp, dirnames, _ in os.walk(root):
        for d in dirnames:
            if d.endswith(("_description", "_moveit_config")):
                robot = d.rsplit("_", 1)[0] if d.endswith("_description") else d[:-len("_moveit_config")]
                cfg_type = "description" if d.endswith("_description") else "moveit_config"
                abs_dir = os.path.join(dp, d)

                pkg_root = find_ros_package_root(abs_dir)
                if pkg_root is None:
                    continue  # non è dentro un package ROS 2 valido
                pkg_name = get_package_name(pkg_root)

                # filtro aggiuntivo di verifica presenza cartella config
                if cfg_type == "moveit_config":
                    config_dir = os.path.join(find_valid_moveit_cfg(abs_dir), "config")
                    if not os.path.isdir(config_dir):
                        continue  # non è una vera cartella di configurazione MoveIt 2

                robots[(pkg_name, robot)][cfg_type] = abs_dir

    return OrderedDict(sorted(robots.items(), key=lambda kv: (kv[0][0], kv[0][1])))


# ───────────────────────────────────────────────
# 2) Analisi di ogni (package, robot)
# ───────────────────────────────────────────────
def analyze(pkg, robot, desc_path, cfg_path):
    # se non c'è un package *_description* useremo il fallback nel moveit_config
    if desc_path is None and cfg_path is None:
        print(f"[SKIP] {pkg}/{robot}: né descrizione né moveit_config trovati.")
        return


    print(f"\n=== Package: {pkg} | Robot: {robot} ===")

    urdfs = find_urdf_xacro_files(desc_path or "", fallback_moveit_cfg=cfg_path)

    if not urdfs:
        print("  ⚠ Nessun URDF/XACRO trovato")
        return

    joints_urdf, prefixes = extract_joints_and_prefixes(urdfs)
    print(f"  Prefissi/namespaces: {', '.join(sorted(prefixes)) or '—'}")

    if cfg_path is None:
        print("  ⚠ Nessun *_moveit_config corrispondente")
        return

    joints_cfg = collect_joints_from_moveit_config(cfg_path)
    missing = sorted(joints_urdf - joints_cfg)

    if missing:
        print("  Joint presenti nell’URDF ma assenti nei config MoveIt 2:")
        for j in missing:
            print(f"    • {j}")
    else:
        print("  ✔ Tutti i joint URDF sono referenziati nei file MoveIt 2.")


# ───────────────────────────────────────────────
# 3) Main
# ───────────────────────────────────────────────
if __name__ == "__main__":
    ROOT = "."            # cambia se necessario
    robots = discover_robot_packages(ROOT)

    if not robots:
        print("❌ Nessun package *_description o *_moveit_config trovato.")
        exit(1)

    print(f"Trovate {len(robots)} coppie (package, robot).")
    for (pkg, robot), paths in robots.items():
        analyze(pkg, robot, paths["description"], paths["moveit_config"])
