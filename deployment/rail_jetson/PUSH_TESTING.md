# WSL Push and Jetson Version Preview

## Scope (2026-09-22)

Implemented: Push in the WSL D-ring, rail and landing-gear recording tabs;
immutable module packages; SSH upload with Jetson hash acknowledgement;
Jetson-hosted version listing, simulated activation and rollback.

The existing WSL Execute, gateway, DDS network and hand-control paths are
unchanged. Push never acquires authority, enables outputs, or starts motion.
The Web service does not connect to ROS/DDS. Its task, recovery, initialization
and ownership controls are SIMULATED. This is not the production cloud endpoint.
No robot boot service has been installed.

The separate headless HTTP adapter has now been tested on Jetson with an offline
transport, without a browser. See `HEADLESS_TESTING.md`. Its gear commands are
four independent segments; the final upstream gear field names remain undecided.
Web state, activation and recovery are not connected to a real output owner.

## Test Without Motion

1. Reload the WSL task application when it is safe to do so. Do not kill a
   controller supporting the arms merely to load the new buttons. This change
   is Python-only; rebuilding ROS packages is unnecessary. Existing running
   Python panels will not display new controls until reopened.
2. In one recording module, choose the desired slot and generate its normal
   executable RViz plan. Do not click Execute for this upload test.
3. Click **Push to Jetson** in that module. Confirm the destination action and
   exact source session shown in the dialog. Only PLAN_READY is eligible;
   selection, configuration, or generation changes during freezing abort Push.
4. Wait for the Jetson receipt showing version and SHA256. Uploaded plans are
   explicitly marked planned, not physically validated. Upload never enables
   the version. A D-ring segment-only plan has a separate binding from its
   complete cycle; a landing-gear segment is never mislabeled a complete cycle.
5. Open the Jetson preview through an SSH tunnel from WSL:

   ```bash
   python3 /home/syf/g1_upper_body_ws/scripts/open_jetson_web.py
   ```

   Open `http://127.0.0.1:8791` in the Windows or WSL browser. The process runs
   on Jetson; this address is its SSH tunnel, not a WSL execution service.
   Use `--local-port 8792` if 8791 is occupied by a different service.
6. Refresh the version list. Check the slot, source session, scope and planning
   status. Apply to the SIMULATOR. A module patch preserves all other actions.
   Switching requires idle simulated Jetson ownership and closes task admission
   until simulated initialization. Test one selected action, then rollback.

The displayed baseline `baseline-20260921-v2` is an offline acceptance fixture
from September 21, not an implicit choice of the user's final production slots.

## Transfer Failure

Local immutable bundles are in `data/action_releases/<version>/bundle.json`.
Receipts, including failed or unconfirmed transfers, are in
`data/jetson_push_receipts/<version>.json`. Failed transfer leaves the current
WSL plan unchanged. A long upload may outlast the normal WSL plan timeout;
that does not change the already frozen bundle or authorize execution.

Retry the SAME bundle (no new planning or compilation):

```bash
cd /home/syf/g1_upper_body_ws
python3 deployment/rail_jetson/upload_release.py \
  data/action_releases/<version>/bundle.json \
  --target unitree@192.168.0.249 \
  --root /home/unitree/rh56dfx/workspace/execution/action_runtime_v2
```

An identical version/hash is idempotent; a conflicting version is rejected.
For address changes edit `config/jetson_push.json`. No password is stored;
the existing SSH key and strict host verification are used. Never disable
host-key checks to make an upload succeed.

## Jetson Files

Root: `/home/unitree/rh56dfx/workspace/execution/action_runtime_v2`.

- `data/action_releases`: verified packages, including inactive module patches.
- `data/web_preview/preview.sqlite3`: SIMULATOR state and task IDs only.
- `data/web_preview/server.log`: Web process log.
- `data/web_preview/process.json`: exact preview process identity.
- `runtime_manifest.json`: staged program hashes; controller launch disabled.

The Web preview is a user process, not a boot service. Its manager reuses an
existing process. A process restart does not silently replay tasks or reacquire
ownership; use its SIMULATED recovery flow if the previous state was retained.

## Verification and Remaining Work

The publisher distinguishes successful historical execution evidence from
newly planned snapshots. The frozen arm/time arrays are retained exactly.
Offline local joins reuse the WSL helpers. Jetson's older SciPy matrix method
name is adapted only in the staged model copy, without changing WSL source.

The generic headless sequence executor is unit-tested through a fake transport;
it is NOT connected to a commissioned robot transport. Actual arm/hand execution,
hardware completion callbacks, real Jetson/WSL output handoff and simultaneous
boot preparation remain deployment gates. Web success does not certify them.

Before any later physical commissioning, verify the selected station/stance,
support, sole command owner, immediate stop method and exact release version.
Do not enable Jetson motor output while the WSL gateway is publishing.
