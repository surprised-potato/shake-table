# Shake Table Control

A browser app for driving a two-axis servo shake table from an Arduino Uno or
Nano. It talks to the board over USB with Web Serial, so it needs **Chrome or
Edge on a desktop** (Firefox and Safari don't support Web Serial).

## Hardware

| Part | Pin |
|---|---|
| X-axis servo (servo 1) signal | D9 |
| Y-axis servo (servo 2) signal | D10 |
| Onboard LED | D13 |

- The servos are **270°** types driven with 500–2500 µs pulses. The table is level at **135°**.
- Each servo arm is 10 mm long (you can change this in the app). Its tip pushes the table along that servo's axis.
- Power the servos from a separate 5 V supply, and connect that supply's GND to the Arduino's GND.

## Flashing the firmware

The firmware is in `firmware/`:

- `led_serial.c`: plain AVR C, the version that's tested and on the board.
- `led_serial.ino`: the same behaviour for the Arduino IDE, using the Servo library. It hasn't been compiled.

**Arduino IDE:** open `led_serial.ino` and upload it to an Uno, or to a Nano with the ATmega328P processor.

**Command line**, using the avr-gcc and avrdude that come with the Arduino IDE (replace `COM5` with your port):

```bash
avr-gcc -mmcu=atmega328p -Os -o led_serial.elf firmware/led_serial.c
```

```bash
avr-objcopy -O ihex -R .eeprom led_serial.elf led_serial.hex
```

```bash
avrdude -C avrdude.conf -p atmega328p -c arduino -P COM5 -b 115200 -D -U flash:w:led_serial.hex:i
```

Uploading fails with *Access is denied* while the app (or the IDE's Serial
Monitor) is connected. Click **Disconnect** first.

## Using the app

Open the page in Chrome or Edge, click **Connect**, and choose the
**USB-SERIAL CH340** port (or your board's port). Connecting resets the board,
so controls unlock after about 2 seconds.

### Table position pad
Click or drag on the ±10 mm grid, and the table glides there at the **Move speed**.
Arrow keys nudge the target by 0.5 mm (2 mm with Shift); **0** or **Centre** goes back to the middle.
The solid trail is the measured motion. The dashed line is the loaded time series.

### Time series player
Plays a recorded or calculated motion in real time.

- **Data (t, x, y):** paste rows, or open a .csv or .txt file. `t` is in seconds and must increase. `x` and `y` are displacements in mm, cm or m.
  Columns can be separated by commas, semicolons, tabs or spaces. A header row and lines starting with `#` are skipped.
  Values between samples are filled in by straight-line interpolation. For an earthquake record, use the **displacement** history.
  ```
  t_s, x_mm, y_mm
  0.00, 0.0, 0.0
  0.05, 1.2, -0.4
  ```
- **Functions:** type `x(t)` and `y(t)` in mm, with `t` in seconds, and set a duration. For example, `5*sin(2*pi*1.5*t)`, or
  `5*noise(4*t)*min(t/2, 1)` for random shaking that ramps up.
  Available: `+ - * / ^`, `pi`, `E`, `sin cos tan asin acos atan atan2 sinh cosh tanh exp log log10 sqrt abs sign floor ceil round min max pow hypot`,
  plus `noise(u)` (smooth random wobble between −1 and 1), `step(u)` and `clamp(v, a, b)`.
- **Amplitude %** scales the motion and **Speed %** changes how fast it plays. The app warns about anything beyond ±10 mm (it's cut off at the edge) and about anything needing more than about 500°/s of servo speed.
- **Play** glides the table to the start point, then streams a target every 20 ms (50 Hz) or 40 ms (25 Hz).
  Keep the tab visible while playing, since browsers slow down hidden tabs.

### Earthquake (back-and-forth)
The board sweeps each axis between a min and max angle at a constant angular
speed. Give the two axes slightly different speeds for irregular, quake-like motion.

### Kinematic model
- Displacement: `x = r · sin(θ − 135°)`.
- Angle for a target: `θ = 135° + asin(x / r)`.
- Table speed: `v = r · ω · cos(θ − 135°)`.
- Angles are limited to **45°–225°**. At those limits the arm is fully out to one side. Any further and it would swing back, so the table would reverse while the servo kept turning the same way.

## Serial protocol (57600 baud, one line per command)

| Command | Meaning |
|---|---|
| `1` / `0` / `t` | LED on / off / toggle |
| `A<deg>` / `B<deg>` | jump servo 1 / 2 to an angle |
| `M <a> <b> <deg/s>` | glide to angles (tenths of a degree, e.g. `M 1650 1050 120`) |
| `T <a> <b> <ms>` | timed move: arrive in `<ms>` milliseconds (used for playback, no reply) |
| `OA` / `OB <min> <max> <deg/s>` | earthquake range and speed per axis |
| `GO` / `STOP` | start / stop the earthquake |
| `H9` `L9` `H10` `L10` | pin test: hold a pin HIGH / LOW |
| `?` | report everything |

Replies: `READY`, `ON`/`OFF`, `A135.0`/`B135.0` (positions, streamed 10 times a second while moving),
`P9=S|H|L`, `OA …`, `RUN` / `MOVING` / `STOP`.
