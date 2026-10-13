#!/usr/bin/env python3
"""Generate the swarm block (N drones + monitor) for the Webots world.

    python gen_world.py                      # writes output.txt next to this script
    python gen_world.py -n 8 --range 60      # 8 drones, radio range 60 m
    python gen_world.py --inject ../worlds/mavic_2_pro.wbt

--inject rewrites the world file in place: everything from the first
"DEF DRONE1" to the end of the file is replaced, so the swarm block must be
the LAST thing in the .wbt (it is, in this project). Close the world in
Webots before injecting, otherwise Webots may overwrite the change.
"""
import argparse
import pathlib
import re


def drone(i, gap, radio_range, fixed, slot, beacon):
    x = -gap * (i - 1) + 0.0                      # + 0.0 avoids printing "-0"
    ctrl_args = '["fixed"]' if fixed else "[]"
    beacon_node = ""
    if beacon > 0:
        # visual only (no boundingObject, no physics): a glowing ball above the drone so it
        # can be seen from far away. The monitor recolours it: gold = leader, red = lost.
        beacon_node = f'''
    Transform {{
      translation 0 0 {beacon + 0.1:g}
      children [
        Shape {{
          appearance DEF BEACON{i} PBRAppearance {{
            baseColor 0.1 0.7 1
            emissiveColor 0.1 0.7 1
            emissiveIntensity 1
            roughness 1
            metalness 0
          }}
          geometry Sphere {{
            radius {beacon:g}
            subdivision 2
          }}
        }}
      ]
    }}'''
    return f'''DEF DRONE{i} Mavic2Pro {{
  translation {x:g} 0 0.1
  name "drone{i}"
  controller "swarm_drone"
  controllerArgs {ctrl_args}
  {slot} [
    Emitter {{
      channel 1
      range {radio_range:g}
    }}
    Receiver {{
      channel 1
    }}{beacon_node}
  ]
}}
'''


def monitor(n, fixed):
    args = f'["{n}", "fixed"]' if fixed else f'["{n}"]'
    return f'''Robot {{
  name "monitor"
  children [
    Receiver {{
      channel 1
    }}
  ]
  controller "monitor"
  controllerArgs {args}
  supervisor TRUE
}}
'''


def build(n, gap, radio_range, fixed, slot, beacon):
    parts = [drone(i, gap, radio_range, fixed, slot, beacon) for i in range(1, n + 1)]
    parts.append(monitor(n, fixed))
    return "".join(parts)


def inject(world_path, block):
    text = pathlib.Path(world_path).read_text()
    cut = text.find("DEF DRONE1 ")
    if cut != -1:
        text = text[:cut]
    # the monitor needs to find the Viewpoint to move the camera
    text = re.sub(r"^Viewpoint \{", "DEF VIEWPOINT Viewpoint {", text, flags=re.M)
    text = text.rstrip("\n") + "\n\n" + block
    with open(world_path, "w", newline="\n") as f:
        f.write(text)


def main():
    here = pathlib.Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", type=int, default=5, help="number of drones (default 5)")
    ap.add_argument("--gap", type=float, default=6.0,
                    help="initial spacing along -x in metres (default 6)")
    ap.add_argument("--range", type=float, default=300.0, dest="radio_range",
                    help="emitter range in metres (default 300). A long line of drones can be\n                    well over 100 m end to end; lower it only to test lost links")
    ap.add_argument("--fixed-leader", action="store_true",
                    help="disable failover: drone1 is always the leader")
    ap.add_argument("--slot", default="bodySlot",
                    help="Mavic2Pro field that holds the Emitter/Receiver (default bodySlot)")
    ap.add_argument("--beacon", type=float, default=0.7,
                    help="radius in metres of the glowing ball above each drone (0 = none)")
    ap.add_argument("--out", default=str(here / "output.txt"))
    ap.add_argument("--inject", metavar="WORLD.wbt",
                    help="also replace the swarm block inside this world file")
    args = ap.parse_args()

    if args.n > 16:
        ap.error("at most 16 drones: the controller has 16 altitude layers (LAYERS)")
    block = build(args.n, args.gap, args.radio_range, args.fixed_leader, args.slot, args.beacon)
    with open(args.out, "w", newline="\n") as f:
        f.write(block)
    print(f"wrote {args.out}")
    if args.inject:
        inject(args.inject, block)
        print(f"injected into {args.inject}")


if __name__ == "__main__":
    main()
