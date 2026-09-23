# Security policy

Slixer moves a robot arm. Bugs that let the arm move when it shouldn't, keep moving after STOP, or be
driven by someone who shouldn't be driving it are treated as security issues, not just bugs.

## Reporting a vulnerability

**Please don't open a public issue for anything security-sensitive.** Report it privately through the
repository's **Security** tab (*Report a vulnerability*), or contact the maintainers, `LaingLab` on GitHub.
If there's no answer within 7 days, you may open a public issue.

Please include what happens, how to reproduce it, the version (`uv run slixer/run.py --version`), and the
platform (OS, Python version, and the firmware on the boards if it's relevant).

## Supported versions

Slixer is in beta. Fixes go into the latest release only.

## What to know when deploying it

- **The page has no login.** Anyone who can reach it can drive the arm and upload files. That's why it
  serves this PC only (`127.0.0.1`) unless it's started with `--host 0.0.0.0`. Only do that on a network
  you trust.
- **The arm's Wi-Fi link has no password of its own.** The boards take poses as plain UDP on port 50101
  from anything on the same network. The Wi-Fi password is the only protection, so keep the arms on a
  network where everyone who can join may also move them.
- **Wi-Fi passwords** go in each sketch's `wifi_config.h`, which git ignores. Don't commit one, and check
  before sharing a copy of the firmware folder.
- **Imported STL files** are parsed on the PC with trimesh. Only import files you trust.

## Out of scope

- Anything that needs physical access to the arm, the boards or the PC.
- The absence of collision checking: imported parts are drawn, not felt, and the documentation says so.
