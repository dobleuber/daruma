// Common Mini16 parameters (mm). The whole design depends on these values.

// ---------- Printing ----------
CL   = 0.4;   // fit clearance between parts (0.3-0.4 depending on the printer)
GAP  = 1.0;   // minimum gap between parts that move relative to each other
T    = 3.0;   // structural arms and plates (PETG, 4 perimeters with a 0.4 nozzle = 1.6 mm + infill)
TF   = 2.0;   // fixed plates resting on the servo face: they must be thinner than how far
              // the disc sticks out (2.6 mm) so they do not rub the arm rotating on top

// ---------- Feetech STS3215 servo (official Waveshare/Feetech "SCS215" drawing) ----------
S_SHORT  = 10.11;  // from the axis to the short end
S_LONG   = 35.11;  // from the axis to the long end (45.22 total)
S_W      = 24.72;  // width
S_CASE   = 32.0;   // case thickness (between the faces with discs)
S_DISC_D = 19.2;   // diameter of the discs (output and passive)
S_DISC_Z = 37.25 / 2;  // outer face of each disc from the mid-plane
S_HORN_R = 7.0;    // radius of the disc's 4 M2 hole circle
M2_HOLE  = 2.4;    // clearance hole for an M2 screw
HORN_CENTER = 6.5; // clearance for the head of the disc's central M3 screw
// M2 case holes (on both faces), measured from the axis along the body.
// Output face: 8.3 and 32.75; passive face: 8.3 and 29.0. All three are drilled on both faces
// so the parts work on the left leg and on the right one (mirrored).
S_CASE_HOLES_X = [8.3, 29.0, 32.75];
S_CASE_HOLE_Y  = 10.25;
S_CORNER_R = sqrt(S_SHORT * S_SHORT + (S_W / 2) * (S_W / 2));  // 16.0: corner of the short side

// ---------- Leg: distances between axes (passed to the simulation model) ----------
R_CB    = S_CORNER_R + GAP;          // inner radius of the crossbar of each "U"
ARM_R   = 11.0;                      // radius of the rounded end of the arms (covers the disc)
D_YAW_ROLL   = T + R_CB + 5;         // yaw servo disc face -> roll axis
                                     // (+5 mm: the roll servo, lying outwards, can tilt ±29°)
D_ROLL_PITCH = S_W / 2 + 3.5 + S_W / 2;   // roll axis -> pitch axis (roll servo above the pitch servo)
D_THIGH = R_CB + T + GAP + S_W / 2;  // hip pitch axis -> knee axis
// Ankle pitch servo in the shin:
//   "fwd": lying forwards -> short shin (33 mm); the knee reaches ~83°
//   "up":  standing along the shin -> 56 mm shin; the knee reaches ~100°
SHIN_SERVO = "fwd";
D_SHIN  = SHIN_SERVO == "up" ? R_CB + T + GAP + S_LONG : R_CB + T + GAP + S_W / 2;
D_ANKLE = D_THIGH;                   // ankle pitch axis -> ankle roll axis
FOOT_CLEAR = 27.5;                   // ankle roll axis -> top face of the foot plate
                                     // (lets the foot tilt ±29° without touching the servo, which sticks out outwards)
PLATE_T = 3.0;                       // foot plate (PETG)
SOLE_T  = 4.0;                       // sole (TPU 95A)
D_SOLE  = FOOT_CLEAR + PLATE_T + SOLE_T;   // ankle roll axis -> sole

HIP_Y   = 30.0;                      // lateral offset of each leg from the center
FOOT_X0 = -38;  FOOT_X1 = 48;        // foot length (8.6 cm), x forward
FOOT_HW = 22;                        // half width of the foot (4.4 cm)
ROCKER_R = 6;                        // radius of the curved sole at the toe, heel and sides

// ---------- Foot contact sensors (SS-10 type microswitch: MEASURE the one you buy) ----------
SW_L = 13.0; SW_W = 6.0; SW_H = 6.5;   // body
SW_PIN = 2.0;                          // mounting pins (spacing SW_PIN_DX)
SW_PIN_DX = 6.5;
SW_X = [FOOT_X1 - 11, FOOT_X0 + 11];   // toe and heel

// ---------- M3 brass inserts ----------
INSERT_D = 4.0;   // hole for an M3 insert (check with the insert manufacturer)
INSERT_L = 5.0;
INSERT_BOSS = 7.5;
M3_HOLE = 3.4;

$fn = 48;
