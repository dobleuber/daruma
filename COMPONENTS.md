# Mini16 components

Decisions made: **Orange Pi Zero 3 + USB camera**, **2x high-drain 18650**,
**Feetech SCS0009 in the head**, **VL53L5CX/VL53L8CX ToF sensor** and **4 foot contact sensors**.
All of this is already reflected in the simulation model (`model/mini16.xml`).

## List

| Qty. | Component | Key data | Mass | Approx. price | Notes |
|---|---|---|---|---|---|
| 12 | Feetech **STS3215 7.4 V** | 19.5 kg·cm, TTL bus, magnetic encoder, 45.2 x 24.7 x 35 mm | 55 g each | ~14 € each | legs; stall current 2–2.5 A each |
| 4 | Feetech **SCS0009** | 2.3 kg·cm at 6 V, 4–7.4 V, TTL bus, 23.2 x 12.1 x 25.25 mm | 11–13 g each | to be confirmed | neck and head; **cannot** be powered directly from the battery |
| 1 | **Orange Pi Zero 3** | H618 4 cores, up to 4 GB, Wi-Fi, ~50 x 55 mm, 5 V input | ~25 g (est.) | you already have it | no camera connector: camera over USB |
| 1 | Compact **USB (UVC)** camera | ~32 x 32 mm board, ~70° field of view | ~6 g | 10–20 € | goes on the front of the head |
| 2 | **High-drain 18650** cell (e.g. Molicel P30B) | 3000 mAh, 30 A | ~46 g each | ~5 € each | in series (2S = 7.4 V nominal, 8.4 V charged) |
| 1 | 2x18650 holder + **2S BMS** (≥ 15 A) | charge/discharge protection and balancing | ~10 g | ~13 € | |
| 1 | Switch + XT30 connector | | ~5 g | ~6 € | |
| 1 | **UBEC 5 V / 3 A** | for the Orange Pi and the sensors | ~10 g | ~4 € | |
| 1 | **6 V / ≥ 3 A buck regulator** | for the 4 SCS0009 | ~5 g | 3–5 € | |
| 1–2 | Electrolytic capacitor **1000–2200 µF / 16 V** | next to the servo power input | ~3 g | ~1 € | |
| 2 | Servo bus adapter (Waveshare Bus Servo Adapter A) | USB or UART ↔ half-duplex TTL bus | ~8 g each | ~5 € each | one for the legs (7.4 V) and one for the head (6 V) |
| 1 | **BNO055** IMU | on-board sensor fusion, I2C or UART | ~3 g | 1–40 € | mounted level between the hips |
| 1 | **VL53L5CX or VL53L8CX** ToF sensor | 8x8 distance matrix, I2C | ~2 g | 15–25 € | on the front of the torso, looking 45° down at the floor |
| 4 | **SS-10** microswitch | foot contact sensor (toe and heel of each foot) | ~1 g each | ~1 € each | alternative: FSR sensors |
| — | Mechanical | 683ZZ / MR105ZZ bearings, M3 brass inserts, M2/M3 screws, PETG or PLA, TPU 95A | — | ~25 € | |
| 1 | 2S balance charger (or 2S USB-C charging module) | | — | 5–25 € | |

**Approximate total (without the board):** ~270–330 €.

## Wiring

```
2x 18650 (2S) ─ 2S BMS ─ switch ─┬─ capacitor ─ bus adapter #1 ─┬─ left leg (6x STS3215)
                                 │   (7.4 V direct)              └─ right leg (6x STS3215)
                                 ├─ 6 V buck ─ bus adapter #2 ─ 4x SCS0009 (neck/head)
                                 └─ 5 V UBEC ─ Orange Pi Zero 3
                                                ├─ USB: camera
                                                ├─ UART or USB: bus adapters #1 and #2
                                                ├─ I2C: BNO055 + VL53L5CX
                                                └─ GPIO: 4x SS-10 (toe/heel of each foot)
```

- One 3-wire harness per leg (the servos are daisy-chained).
- The Orange Pi Zero 3 has a single USB-A port, used by the camera: the bus adapters
  connect over UART on the 26-pin header, or through a small USB hub.

## Weight (according to the model)

| Group | Mass |
|---|---|
| 12 STS3215 | 660 g |
| 4 SCS0009 | 48 g |
| 2x18650 battery + holder | 98 g |
| Orange Pi + regulators + adapters + BMS + wiring | ~75 g |
| Sensors (IMU, ToF, camera, foot contact) | ~15 g |
| Plastic (CAD legs ~173 g, torso, head) | ~280 g |
| **Total** | **~1.18 kg** |

It is ~180 g above the 1 kg target of the specification. The leg servos are
60 % of the weight; some can be trimmed with thinner shells or a smaller battery (at the cost of
runtime), but with 12 STS3215 it is hard to go below ~1.05 kg.

## To check before buying

- **STS3215 voltage:** the charged battery gives 8.4 V and the servo's stated range is 6–7.4 V.
  Open Duck powers them directly from 2x18650; confirm it in the Feetech datasheet (otherwise, a
  high-current 7.4 V regulator).
- **SCS0009:** confirm the price and that the bus adapter and the Feetech library handle its
  protocol (SCS), which is from the same family as the STS one.
- **Orange Pi Zero 3:** confirm in the official schematic which pins are UART and I2C (the sources
  disagree) and the real power draw.
- **BNO055:** some boards have I2C problems; if that happens, use its UART mode.
- **VL53L5CX/L8CX:** needs its firmware loaded over I2C at startup (the ST library does it).

## Sources

- STS3215: [Core Electronics](https://core-electronics.com.au/feetech-sts3215-smart-servo.html),
  [RobotShop](https://www.robotshop.com/products/magnetic-encoding-servo-sts3215-74v-19kgcm)
- SCS0009: [OpenELAB](https://openelab.io/products/seeed-studio-feetech-scs0009-ultra),
  [Pakronics](https://www.pakronics.com.au/products/feetech-scs0009-ultra-compact-servo-6v-2-3kg-for-amazing-hand-ss100067882)
- Orange Pi Zero 3: [CNX Software](https://www.cnx-software.com/2023/07/03/orange-pi-zero-3-allwinner-h618-sbc)
- Open Duck Mini v2 bill of materials (reference prices):
  [Google Sheets](https://docs.google.com/spreadsheets/d/1gq4iWWHEJVgAA_eemkTEsshXqrYlFxXAPwO515KpCJc)
