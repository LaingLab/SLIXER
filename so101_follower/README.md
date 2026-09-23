# SO-101 wireless teleoperation, two XIAO ESP32-C3 boards

The leader arm's XIAO reads its six joints 50 times a second and broadcasts them by radio (ESP-NOW). The
follower arm's XIAO drives its servos to match. No computer, no network, no pairing: the two boards find
each other by themselves, and either can be restarted at any time.

Each XIAO plugs into its arm's Seeed *Bus Servo Driver Board for XIAO*, which powers it from the arm's own
supply. Calibration is read from the servos at every power-up (lerobot-calibrate stores it there), so
recalibrating never needs a re-flash. Joint mapping matches `lerobot-teleoperate` to within one encoder step.

## 1. Calibrate first

Once a board is switched to XIAO mode, lerobot on the PC can't reach that arm until you switch it back
(section 9). Do any `lerobot-calibrate` runs before converting. For the "middle" step put every joint
mid-range, and push each joint all the way to both stops during the sweep.

## 2. Switch both driver boards to XIAO mode

On each board:

1. Pull off the 2-pin **jumper cap** on the front.
2. On the back, open the solder jumpers **JP1** and **JP2**: no continuity between each pair of pads.
   (Early boards have two small resistors, R12 and R18, there instead: remove them.)
3. Solder headers to the XIAO and plug it in, USB-C end matching the outline printed on the board.

In USB mode the board's USB chip sits on the same D6/D7 wires as the XIAO, and the two fight over the line.

## 3. Flash

Arduino IDE 2, **Tools > Board > esp32 > XIAO_ESP32C3**, USB CDC On Boot enabled (the default).

- Open `so101_leader/so101_leader.ino`, select the leader's XIAO port, Upload.
- Open `so101_follower/so101_follower.ino`, select the other XIAO port, Upload.

The two sketches share the same headers; the folder decides which board does what. Built and checked with
esp32 core 3.3.8 and 3.3.12. For Wi-Fi, set it up first (section 7); without it the boards use their direct
radio link only, which is all teleoperation needs.

## 4. Use

Power both arms, in any order. Within a second or two the follower glides to the leader's pose and then
tracks it. Nothing moves until the follower has checked its own arm and is receiving the leader's pose.

The follower has no built-in LED. Wire one from **D10** through a ~330 ohm resistor to GND for status:

| LED | Meaning |
|---|---|
| slow blink (1 Hz) | arm checked, waiting for the leader |
| medium blink | gliding to the leader's pose |
| steady on | following |
| fast blink | leader data stopped: holding its pose, torque on |
| repeating blink count | a problem: 1 = no reply from a servo, 2 = no usable calibration, 3 = a joint outside its range, 4 = torque refused |

The follower holds its pose while the leader is off. Switch the follower off first, or support the arm.

Wrist roll is the one joint that can reach the encoder's wrap point: past about half a turn from its
calibration orientation the follower stops at its limit instead of spinning round.

## 5. Watching what happens

Open the Serial Monitor on the **leader** (any baud rate). It prints its own state and the follower's,
which the follower reports over the radio every 2 seconds:

```
[leader] follower reporting, 1500 poses sent, 15 follower reports, 0 read failures | deg: ...
[follower] following | 98 poses received
```

This matters because the follower's board feeds its XIAO from the arm's supply, which fights the PC's USB:
with 12 V applied, the follower's own USB port usually won't enumerate. The leader's board runs from 5 V and
doesn't have that problem.

## 6. Re-recording a joint's travel without a PC

lerobot-calibrate writes each joint's travel into the servos, and the servos enforce it: a joint that was
swept too little during calibration can't move beyond what was recorded. You can redo the sweep from the
**leader's** Serial Monitor, with no soldering and no lerobot. Homing offsets (the joint zero points) are
left alone, so this fixes a bad range without redoing a full calibration.

| Key | Effect |
|---|---|
| `h` / `H` | centre the arm on the pose it is in now (leader / follower) |
| `r` / `s` | start / save a sweep of the **leader's** arm |
| `R` / `S` | the same for the **follower's** arm, sent over the radio |
| `x` / `X` | cancel |
| `?` | list the keys |

**Centring (`h`/`H`) is only needed when a joint's travel crosses the encoder's 0/4095 seam**, which shows
up as that joint sweeping nearly 4095 steps while the others sweep about 2200. Such a sweep is refused when
you save it, and a range like that already in a servo is reported at start-up (see Troubleshooting), rather
than aiming the joint at a middle it can't reach. Hold every joint near the middle of its travel, press
`h`, then sweep and save. Centring opens the limits right up until you save a
sweep, so if you power off in between, the arm reports no usable calibration and you simply redo it.

The servos only keep new ranges through a power cycle if their EEPROM write-lock is clear, so saving
clears it, writes, reads the values back, and locks again. Power-cycle once afterwards to confirm: the arm
should still report `arm ready`.

Type the key and press Enter. **Support the follower before `H` or `R`**: both switch its torque off
first, so it goes limp (the log says `torque off: support the arm`). While recording, that arm's torque
stays off and the leader stops sending poses, so the follower holds still instead of mirroring the sweep.
Move **every** joint to both of its stops, then save. A sweep that missed a joint, or crossed the seam, is
refused rather than saved, and the recording carries on: sweep again and save, or cancel. After a save the
arm re-checks itself, so the log tells you whether it took.

## 7. Putting the arms on Wi-Fi (for a PC to drive them)

In **both** sketch folders, copy `wifi_config.example.h` to `wifi_config.h`, fill in your network's name
and password, and re-flash. (`wifi_config.h` is ignored by git, so the password stays on your machine.)
Each board then prints `link up (radio + wifi)` and carries the same packets over both paths: the direct
radio link between the boards, and UDP for anything else on the network. Without `wifi_config.h`, or
with the name left empty, the boards use the radio only.

With Wi-Fi up, a PC anywhere the network reaches can watch the arm and drive it -- with
[Slixer](../slixer/README.md), or from your own code using `pc/so101_link.py`:

```
python pc/so101_link.py            # watch the arm: state, joint angles, gripper
python pc/so101_link.py --hold     # hold it where it is, proving the control path
```

No addresses to configure: the arm announces itself once a second and the script learns where it is.
Targets are joint angles measured from the middle of each joint's own travel, the same units the leader
sends. A program's poses are marked as coming from a program, and while they keep coming the follower
holds the leader back; a quarter of a second after the program stops, the leader has the arm again. So
grabbing the leader does **not** take over from a running program -- stopping the program does. With no
leader powered, stopping leaves the arm frozen in place rather than limp.

Press **W** in either board's Serial Monitor to list the networks it can hear, with their signal strength:
the quickest way to tell a wrong password from a board that can't hear the network at all.

**Under WSL**, add this to `C:\Users\<you>\.wslconfig` and run `wsl --shutdown`, otherwise packets from
the arm can't reach it (it also detaches any usbipd devices, so re-attach them afterwards):

```
[wsl2]
networkingMode=mirrored
```

Running the script from Windows instead needs no configuration at all.

## 8. Radio range

The XIAO ESP32-C3 depends on the **external antenna** that came in its box: plug one into the U.FL socket
on each board, and keep it clear of the arm's metalwork rather than lying against a bracket. Without it,
range is poor.

The firmware sends at the chip's maximum power and uses Espressif's Long Range PHY at 250 kbps, which is
far more robust than ordinary Wi-Fi rates. **Both boards must run the same firmware version for this** — a
board in long-range mode and one without it cannot hear each other at all.

Still short? Change `kFallbackChannel` in `link.h` (1, 6 and 11 are the usual choices), re-flash both, and
pick one your Wi-Fi router isn't sitting on.

## 9. Back to lerobot on the PC

Unplug the XIAO, re-bridge JP1 and JP2 with solder, and refit the jumper cap. Plug the XIAO back in only
after re-opening them.

## Troubleshooting

- **`no reply from servo N`**: arm power, board not in XIAO mode (JP1/JP2, jumper cap), XIAO orientation, or
  the 3-pin servo lead. To rule out the XIAO itself, take it off the board, join **D6 to D7** with a wire,
  power it by USB and type **`L`** in its Serial Monitor: it sends six bytes and checks they come back.
- **`outside its calibrated range`**: move that joint by hand toward the middle; it re-checks every second.
- **`no usable calibration in servo N`**: that joint's recorded travel is too small to use. Re-record it
  with the keys in section 6, or recalibrate with lerobot.
- **`... has no range yet (limits 0-4095)`**: the servo's limits are wide open, as centring leaves them.
  Record a range with the keys in section 6, or recalibrate with lerobot.
- **`... is nearly a full turn: it crosses the encoder seam`**: that joint's range was recorded across the
  0/4095 seam, so its middle is somewhere it can't reach. Centre (`h`/`H`) and record it again (section 6).
- **The IDE offers "Ozobot circuit kit" instead of the XIAO**: its board entry claims the same generic
  Espressif USB id, so auto-detect matches it first. Select **Tools > Board > esp32 > XIAO_ESP32C3** by
  hand and ignore the mismatch warning; the sketch refuses to build for the wrong chip anyway.
- **Nothing from the follower in the leader's log**: check the follower has power, is flashed with
  `so101_follower`, and that both boards print `radio up` at start-up.
- **The follower joins Wi-Fi with the arm's 12 V unplugged, but not with it on**: noise from the servo
  supply is drowning its Wi-Fi receiver. Press **W**: hearing *no* networks at all with the 12 V on, and
  yours with it off, confirms it. (The board-to-board radio keeps working, because its long-range mode is
  far more robust than Wi-Fi.) Move the antenna away from the servo board and its power wiring, add
  decoupling at the XIAO or ferrites on the power leads, or try a quieter supply.
