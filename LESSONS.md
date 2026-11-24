# What to learn from Microduck and Open Duck

Comparison made against their repositories (MJCF model, constants and training guide of
Microduck; sim2real documentation of Open Duck), not from memory.

## Mass distribution: the main difference

| | Microduck (model) | Mini16 (our CAD) |
|---|---|---|
| Total mass | 737 g (~800 g real) | 1182 g |
| Height | ~25 cm | ~32 cm |
| Servos | 14 XL330 (~18 g) | 12 STS3215 (55 g) + 4 SCS0009 |
| **Legs** | **35 %** (129 g each) | **62 %** |
| Torso | 27 % (199 g) | ~30 % |
| **Neck + head** | **38 %** (head alone 189 g) | **8 %** |
| Motors per leg | 5 (no ankle roll) | 6 |

Microduck has light legs and almost 40 % of its mass up high, in a movable head. With that
head the robot balances itself: moving it shifts the center of mass without moving the legs. On Mini16
it is the other way around. The legs weigh so much that every step moves almost the whole robot, and to
compensate I added the ankle roll, which adds two more heavy servos.

## Lessons

1. **Small servos in the legs.** An XL330 weighs a third of an STS3215. Open Duck uses
   STS3215, but it is 42 cm tall: big servos call for a big robot.
2. **5 motors per leg are enough** if balance comes from the controller, not the mechanics. Neither
   of the two has ankle roll.
3. **Heavy, movable head (4 joints)** as a counterweight. The battery or the board can
   go in the head to give it useful mass.
4. **Rest pose:** knees almost straight, hip pitch −26°, ankle +26°. The torso is
   shifted 5 mm forward so that the center of mass sits over the ankle axis.
5. **Balance comes from RL, not from a scripted gait.** Open Duck imitates a reference
   gait (Disney BDX style) and the RL policy learns to correct it. Microduck trains PPO with 4096
   parallel simulations on a GPU, 1–2 h.
6. **Faithful motor model (BAM)** and randomization in the simulation: battery voltage,
   delay, friction, ±1° gear backlash. Open Duck publishes the BAM parameters of the
   STS3215 at 7.4 V.
7. **Evaluate designs with RL.** My mistake was comparing designs with the scripted gait, which
   favors ankle roll. With RL, the 5-motor, heavy-head design probably wins.

## Possible directions

- **A. Microduck style:** small servos (XL330 or an equivalent Feetech) in the legs, 5 per
  leg, heavy head with 4 joints, ~25 cm and ~800 g.
- **B. Open Duck style:** keep the STS3215, 5 per leg, and scale up to ~40 cm with a heavy
  head.

In both cases: model the motor with BAM and train with RL on the NVIDIA GPU (short
training, 30–60 min), instead of hand-tuning a scripted gait.
