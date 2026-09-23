# Contributing to Slixer

Thanks for your interest. Slixer moves a real robot, so the bar for a change is: would you trust it with
your hand near the arm?

## Setting up

```bash
git clone https://github.com/LaingLab/SLIXER.git
cd SLIXER
uv sync --extra vision      # or plain `uv sync` if you won't touch the YOLO side
uv run pytest               # all of it should pass on a clean checkout; if not, open an issue
```

The tests need no hardware: they drive `slixer/tests/fake_follower.py`, which runs the follower firmware's
own logic, and `slixer/tests/fake_camera.py`. `slixer/README.md` shows how to try the whole app against
both by hand.

## Before you start

**Open an issue first** for anything bigger than a small fix, so we can agree on the approach before you
spend time on it.

## Things that are easy to get wrong

- **The firmware headers are shared.** `so101_leader/` and `so101_follower/` each carry copies of
  `feetech.h`, `link.h`, `ranges.h`, `serial_port.h` and `teleop_math.h`, because the Arduino IDE builds a
  sketch from its own folder. Change both copies, and keep them identical (`diff` them).
- **The imitation follower copies the firmware.** When the follower's behaviour or its packets change,
  change `slixer/tests/fake_follower.py` with it, or the tests go on testing an arm that no longer exists.
- **Only `session.py` sends to the arm.** Keep it that way: it is where the modes and safety rules live.
- **Never commit** a `wifi_config.h`, anything in `slixer/models/` but its README, or your own programs,
  calibration and scene. `.gitignore` covers them; check `git status` anyway.

## Pull requests

- One change per pull request, with a test that fails without it.
- `uv run pytest` passes.
- User-visible changes get a line in `CHANGELOG.md` under *Unreleased*.
