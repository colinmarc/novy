# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///

"""Generate a KiCad footprint for the Elma Type 04 1x12 rotary switch.

The pin circle radius is estimated at 12.5mm from the datasheet
drilling diagram. Verify with calipers on the actual switch and
adjust PIN_CIRCLE_R if needed.

Pin numbering follows the datasheet 1x12 (30° indexing) diagram,
viewed from the shaft end and mirrored for PCB component side.
"""

import math

# --- Parameters (adjust as needed) ---
PIN_CIRCLE_R = 24.1 / 2   # mm, radius of the 12 signal pins (outer ring)
PIN_DRILL = 1.3           # mm, signal pin hole
PIN_PAD = 2.0             # mm, signal pin pad diameter
MOUNT_HOLE = 10.5         # mm, shaft mounting hole (M10 + clearance)
NUM_POSITIONS = 12
INDEX_ANGLE = 30.0        # degrees between positions
FIRST_PIN_OFFSET = -12.5  # degrees, pin 1 offset from horizontal
                          # (shaft-end view, clockwise = negative)


# The drilling diagram is "view from the shaft end," which is the
# same as looking at the PCB from the component side. No mirroring
# needed. In KiCad, 0° = right (+X), Y increases downward.


def pad(number, x, y, drill, size, shape="circle", pad_type="thru_hole"):
    return f"""  (pad "{number}" {pad_type} {shape}
    (at {x:.4f} {y:.4f})
    (size {size:.2f} {size:.2f})
    (drill {drill:.2f})
    (layers "*.Cu" "*.Mask")
  )"""


def npth(x, y, drill):
    return f"""  (pad "" np_thru_hole circle
    (at {x:.4f} {y:.4f})
    (size {drill:.2f} {drill:.2f})
    (drill {drill:.2f})
    (layers "*.Cu" "*.Mask")
  )"""


def main():
    pads = []
    silk = []

    # 12 signal pins on the circle.
    # Shaft-end view: pin 1 at FIRST_PIN_OFFSET, going clockwise.
    # PCB component-side view (mirrored): negate X, so angles flip.
    for i in range(NUM_POSITIONS):
        # Shaft-end view = component-side view, no mirroring.
        # Clockwise in the diagram = clockwise on the PCB.
        # KiCad Y is inverted (positive = down), so negate sin.
        angle = FIRST_PIN_OFFSET - i * INDEX_ANGLE
        rad = math.radians(angle)
        x = PIN_CIRCLE_R * math.cos(rad)
        y = -PIN_CIRCLE_R * math.sin(rad)
        pads.append(pad(i + 1, x, y, PIN_DRILL, PIN_PAD))

    # Common pin (pin 13) on the inner ring (diameter 14.3mm).
    # At 45° NE in the shaft-end view (no mirroring needed).
    common_r = 14.3 / 2  # inner ring radius
    common_angle = 45.0  # 45° NE
    common_rad = math.radians(common_angle)
    common_x = common_r * math.cos(common_rad)
    common_y = -common_r * math.sin(common_rad)
    pads.append(pad(13, common_x, common_y, PIN_DRILL, PIN_PAD))



    # Silkscreen: circle for body outline.
    body_r = 17.0  # approximate body radius
    silk.append(
        f"  (fp_circle (center 0 0) (end {body_r:.2f} 0) "
        f"(stroke (width 0.12) (type solid)) (layer \"F.SilkS\"))"
    )
    # Pin 1 marker.
    p1_rad = math.radians(-FIRST_PIN_OFFSET)
    p1_x = (PIN_CIRCLE_R + 3) * math.cos(p1_rad)
    p1_y = -(PIN_CIRCLE_R + 3) * math.sin(p1_rad)
    silk.append(
        f"  (fp_circle (center {p1_x:.2f} {p1_y:.2f}) (end {p1_x + 0.5:.2f} {p1_y:.2f}) "
        f"(stroke (width 0.12) (type solid)) (layer \"F.SilkS\"))"
    )

    # Courtyard.
    cy_r = body_r + 1
    silk.append(
        f"  (fp_circle (center 0 0) (end {cy_r:.2f} 0) "
        f"(stroke (width 0.05) (type solid)) (layer \"F.CrtYd\"))"
    )

    # Fabrication layer.
    silk.append(
        f"  (fp_circle (center 0 0) (end {body_r:.2f} 0) "
        f"(stroke (width 0.1) (type solid)) (layer \"F.Fab\"))"
    )

    # Reference and value.
    header = f"""(footprint "Elma_04-1124-20_1x12_30deg"
  (version 20240108)
  (generator "gen_elma04_footprint.py")
  (layer "F.Cu")
  (descr "Elma Type 04 selector switch, 1x12, 30 degree, PCB pins")
  (tags "rotary switch elma 04")
  (property "Reference" "REF**"
    (at 0 -{body_r + 3:.1f})
    (layer "F.SilkS")
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (property "Value" "Elma_04-1124-20"
    (at 0 {body_r + 3:.1f})
    (layer "F.Fab")
    (effects (font (size 1 1) (thickness 0.15)))
  )"""

    content = header + "\n"
    for p in pads:
        content += p + "\n"
    for s in silk:
        content += s + "\n"
    content += ")\n"

    out = "Elma_04-1124-20.kicad_mod"
    with open(out, "w") as f:
        f.write(content)
    print(f"Written to {out}")
    print(f"\nPin positions (component side, mm):")
    for i in range(NUM_POSITIONS):
        angle = FIRST_PIN_OFFSET - i * INDEX_ANGLE
        rad = math.radians(angle)
        x = PIN_CIRCLE_R * math.cos(rad)
        y = -PIN_CIRCLE_R * math.sin(rad)
        print(f"  Pin {i+1:2d}: ({x:7.3f}, {y:7.3f})  "
              f"angle={angle:.1f}°")
    print(f"  Pin 13 (common): ({common_x:.3f}, {common_y:.3f}) r={common_r:.1f}mm")

    print(f"\n⚠ Outer ring diameter=24.1mm, inner ring diameter=14.3mm")
    print(f"  (from datasheet). Common pin angle is assumed — verify")
    print(f"  on the physical switch and adjust if needed.")


if __name__ == "__main__":
    main()
