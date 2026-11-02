# G1 RH56DFX Project Rules

## Scope

- Project root: `/home/unitree/rh56dfx/workspace`.
- Ask before any global environment change or write outside this repository.
- Treat `/home/unitree/unitree_sdk2-main` as a read-only reference unless the
  user explicitly authorizes changes.
- `hand_service` is the only serial hardware owner. All tools and Web control
  must use `HandClient`; do not add another protocol or DDS control path.

## Hardware contract

- Right: FTDI `if01`, RH56 ID 1. Left: FTDI `if02`, RH56 ID 1.
- Always use the stable `/dev/serial/by-id/...` paths in project defaults.
- Official Unitree `q`: 0 closed, 1 open; raw RH56 angle is `q*1000`.
- Order: right `[pinky, ring, middle, index, thumb_bend, thumb_rotation]`, then
  left in the same order.
- Actuator feedback is not an external phalanx-angle measurement. Opening can
  release the passive finger mechanism, whose visible pose depends on gravity,
  orientation and contact.

## Safety

- Do not run motion commands without explicitly warning the user first.
- Respect a requested left-only or right-only test; never probe the other hand.
- Before motion verify mapping, serial access, process ownership, clear workspace,
  robot stability and an immediate stop method.
- A graceful `hand_service` shutdown holds current positions before exit. It
  cannot replace power removal or the robot's physical E-stop.
- Never start `hand_service` as a diagnostic.

## Verification

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
cd build && ctest --output-on-failure
```

Report separately what was compiled, read from hardware, and motion-tested.
