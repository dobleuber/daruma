# Mini16 CAD (OpenSCAD)

Printable leg parts, designed with the dimensions from the official drawing of the
**Feetech STS3215** (body 45.22 x 24.72 x 32 mm, axis 10.11 mm from one end, Ø19.2 mm
discs on both sides with 4 M2 holes on a Ø14 mm circle, M2 case holes at 20.5 mm).

Status: **complete leg** (left and right). Missing: torso and head.

## Parts per leg

| Part | Material | Infill | Approx. mass | What it does |
|---|---|---|---|---|
| `yaw_link` | PETG | 35 % | 7 g | bolted to the disc of the hip yaw servo; "U" over the hip roll servo |
| `roll_link` | PETG | 35 % | 14 g | holds the roll servo (lying outwards) and the pitch servo (lying forwards) |
| `thigh` | PETG | 35 % | 13 g | "U" over the hip pitch servo; cradle for the knee servo (lying backwards) |
| `shin` | PETG | 35 % | 13 g | "U" over the knee; cradle for the ankle pitch servo (lying forwards) |
| `ankle_cross` | PETG | 35 % | 13 g | "U" over the ankle pitch; cradle for the ankle roll servo (lying outwards) |
| `foot` | PETG | 35 % | 19 g | "U" over the ankle roll; plate with pockets for 2 foot contact sensors |
| `sole` | **TPU 95A** | 40 % | 8 g | sole with curved edges (toe, heel and sides) |

Right leg: the same parts mirrored (`right_*.stl`). Total plastic: ~89 g per leg.

**Printing:** 4 perimeters (0.4 mm), 0.2 mm layers, no supports where possible (flat
plates facing the bed). Fit clearance `CL = 0.4 mm` in `lib/params.scad`: lower it to 0.3 if the
printer is accurate.

## Distances between axes (used by the simulation)

| Segment | mm |
|---|---|
| Hip yaw disc → roll axis | 25.0 |
| Roll axis → pitch axis | 28.2 |
| Thigh (pitch → knee) | 33.3 |
| Shin (knee → ankle pitch) | 33.3 (56.1 with `SHIN_SERVO = "up"`) |
| Ankle pitch → ankle roll | 33.3 |
| Ankle roll → sole | 34.5 |
| Lateral offset of each leg from the center | 30.0 |

Collision-free ranges (checked with `check_leg.py`): hip yaw ±34°, hip roll ±29°,
hip pitch −14° to 74°, knee −83° to 6°, ankle −11° to 80°, ankle roll ±29°
(with `SHIN_SERVO = "up"`: knee down to −100° and ankle ±80°, at the cost of a 23 mm longer shin).
The gait used in the simulation causes no collisions between parts, nor between the two legs.

## Fasteners (per leg)

- Servo discs: 4x M2 per disc (2 discs per servo held in a "U"); the servo's central M3
  screw.
- Servo cases: M2 self-tapping screws into the servo holes.
- Sole to foot: 4x M3 x 8 + 4 M3 brass inserts (Ø4.0 hole, 5 mm).
- Foot contact sensors: 2 microswitches with Ø2 pins (**measure the model you buy** and
  adjust `SW_*` in `lib/params.scad`).
- Blue 243 threadlocker only on metal-to-metal joints (central disc screw).

Note: the STS3215 already comes with a passive shaft with its own bearing, so the parts hold the servo
by its two discs and no printed bearing is needed.

## Workflow

```bash
./export.sh            # parts -> stl/ (left, right and servos)
python check_leg.py    # checks collisions over the full range (and --gait: with the gait)
python build_mjcf.py   # regenerates model/mini16.xml with these dimensions, meshes and masses
```

`lib/params.scad` is the single source of the dimensions: after changing something there, run the three steps.
