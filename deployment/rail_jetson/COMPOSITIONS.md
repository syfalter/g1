# Recorded business compositions (candidate contract)

These recipes reference existing immutable action plans. They do not concatenate,
trim, retime or reverse recorded curves. A full release is pinned for the entire
task; later Push operations cannot replace a segment mid-task. This candidate is
not deployed, and the compositions have not been physically validated.

## D-ring: one left + one right = one task cycle

The existing endpoint already supports paired repetitions:

```text
POST /api/v1/tasks
X-API-Key: <existing key>
Content-Type: application/json
```

```json
{"taskId":"dring-pair-001","taskType":"NWS_CYCLE","callbackUrl":"http://SERVER:PORT/callback","parameters":{"count":1}}
```

For count=3, execute left,right,left,right,left,right. Progress increments only
after both actions in a pair succeed. The present bindings are full actions,
including their existing preparation/return paths. The less direct intermediate
path remains intentionally unchanged pending the user's trajectory editing.
Do not strip endpoints or replace them with an unvalidated shortcut.

## Gear: explicit round trips, finish position and exit

New candidate endpoint: POST `/api/v1/gear/tasks`, same headers as above.

```json
{"taskId":"gear-lower-001","callbackUrl":"http://SERVER:PORT/callback","parameters":{"startPosition":"LOWER","roundTrips":2,"finishPosition":"UPPER","exitToAbove":false}}
```

Sequence: ABOVE -> LOWER -> UPPER -> LOWER -> UPPER -> LOWER -> UPPER.
The two complete round trips return to LOWER; the last transfer reaches UPPER.

```json
{"taskId":"gear-upper-001","callbackUrl":"http://SERVER:PORT/callback","parameters":{"startPosition":"UPPER","roundTrips":2,"finishPosition":"LOWER","exitToAbove":true}}
```

Sequence: ABOVE -> UPPER -> LOWER -> UPPER -> LOWER -> UPPER -> LOWER -> ABOVE.
The final recorded exit is `gear_lower_to_above`, not a generated home shortcut.

Fields are required, unknown fields rejected. Positions are LOWER/UPPER;
roundTrips is an integer 0..1000; exitToAbove is a strict boolean. Zero round
trips still executes entry and any requested final transfer/exit. startPosition
means the start of cycling AFTER entry, not a claim about the current robot or
gear state. V1 always includes the recorded ABOVE entry; starting directly in
contact at a rod position without entry is not implemented. Position labels
describe recorded arm segments, not externally sensed gear lock states.

Callbacks use taskType GEAR_SEQUENCE and the normal event shape. Progress stays
0 throughout execution and becomes 1 only after the final transfer and any exit
succeed. RUNNING messages include the current action and its internal stage.
The upstream callback enum must accept GEAR_SEQUENCE and respond with HTTP
success and {"code":0}. Query/cancel use existing POST routes
`/api/v1/tasks/{taskId}/query` and `/api/v1/tasks/{taskId}/cancel`.

Inspect without movement using POST `/api/v1/gear/preview` with ONLY the
parameters object as body. This does not initialize or submit a task. The reply
contains entry/repeat/tail, actionCount, missingActions, selected plan hashes and
release hash. configured=true means bindings exist, NOT physical verification
or readiness. Authorization is still required. Preview creates no task ID.

Legacy GEAR_OPERATION remains rejected: its older preAction/cycleMode fields do
not unambiguously express these requirements. Do not silently reinterpret them.
An upstream adapter may translate agreed business commands into this endpoint.

## Execution and replacement

Every needed action (including exit) is checked before movement. No busy-task
interleaving, implicit initialization, automatic motion retries or continuation
after cancellation/failure is added. Existing fault recovery policy is retained.
The task requires Jetson ownership and initialization, independent of Web/WSL.
Scene clearance and first physical validation remain necessary. The current
runtime can locally align to a recording start; it does not infer contact state.

Rail push/pull and AIR/GROUND mapping are unchanged. The September 24 snapshot's
push/pull failed continuity validation and are not repaired by these recipes.

After the edited D-ring segments are ready, build and publish new immutable
action bindings and validate their transitions. Recipes can retain their business
meaning, but must not be considered physically verified merely because names
are unchanged. No recorded arrays are modified by recipe export.

## Offline alternatives

```bash
python3 deployment/rail_jetson/compositions.py --release data/rail_deployment/panel_snapshot_20260924/bundle.json --output data/rail_deployment/composition_candidates_20260925.json
```

Exports 24 gear variants (both starts, both finishes, hold/exit, 0/1/3 round trips)
plus D-ring 1/3-pair recipes, with source hashes. These are composition manifests,
not independently executable trajectory files and not a hardware authorization.
