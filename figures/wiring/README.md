# SO-101 wiring diagram

The post's isometric wiring figure: the Mac, a USB-C lead to each arm's Seeed servo driver board,
that board's power supply, and six STS3215 servos daisy-chained on one three-wire cable; the
follower's bus alone, then both arms, with a close-up of the board's CH343P USB-to-serial chip.

| File | What it is |
|---|---|
| `build_scene.py` | Builds the scene in Blender, in millimetres, and renders it to two colours per pixel |
| `compose.py` | Writes the labels as a transparent SVG over the render |
| `layout.py` | Where each version's canvas sits: the follower alone, both arms, the chip close-up |
| `gen_wiring.py` | The first, flat-shaded version: a standalone SVG, standard library only |
| `refs/sts3215/` | The STS3215 servo mesh the servos are rendered from, with its licence |

Blender only measures the scene (which part, its light, normal and depth per pixel). From those,
`build_scene.py` works out the 3D arm viewer's look (its palette and joint colours, two tones per
face, heavy ink silhouettes, thin overlap and crease lines) as two colours per pixel and how much
of the first. The dither is left to the page that shows it, which applies the viewer's ordered
dither at the screen's own pixels, so the dots match the arm figure at any size.

```
Blender -b --python build_scene.py -- r-follower.png follower     # or: both, chip
# -> r-follower-a.png (first colour, coverage as alpha), r-follower-b.png, r-follower.json
python3 -c "from PIL import Image; [Image.open(f'r-follower-{k}.png').save(f'follower-{k}.webp', lossless=True, exact=True) for k in 'ab']"
python3 compose.py r-follower.json follower-labels.svg follower
```

Blender needs its GPU, so it won't start inside a sandbox that hides Metal.

## Where the numbers came from

| Part | Value used | Source |
|---|---|---|
| Servo case, wheel, ports | 45.4 × 24.8 × 32.9 mm; two 3-pin ports on the back face | Measured from `refs/sts3215/sts3215_03a_v1.stl` |
| Board | 42 × 25 mm, 5–12 V DC at the screw terminal | [Seeed product page](https://www.switch-science.com/products/11115) |
| Board chips | CH343P; SN74LVC1G126 (SOT-23-5) send buffer, SN74LVC1G125 (SC-70-5) receive buffer | [Seeed schematic](https://wiki.seeedstudio.com/bus_servo_driver_board/); positions on the board are eyeballed from a photo |
| CH343P | QFN16, 3 × 3 mm; TXD pin 4, RXD 5, UD+ 7, UD− 8 | [WCH datasheet](https://static.chipdip.ru/lib/163/DOC045163990.pdf) |
| MacBook Pro | 312.6 × 221.2 × 15.5 mm, two USB-C ports on the left | [Apple tech specs](https://www.apple.com/macbook-pro/specs/) (14-inch) |
| Wire colours | black ground, red power, white data | The usual convention for these servos |

The STL is from TheRobotStudio's [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
(`Simulation/SO101/assets`, commit `a758567`), Apache License 2.0; the licence is alongside it.
