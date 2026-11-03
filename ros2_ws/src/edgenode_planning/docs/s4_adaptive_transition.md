# Adaptive S4 entry and recovery

The curvature limit and steering clamp are unchanged. This change replaces the
fixed-distance lateral profile with geometry-derived transitions, retaining
collision checks, direction continuity, BLOCKED persistence, actual rear-passage
recovery, the S4 3 km/h cap, and Perception self-mask.

## Recorded scene and old generator

`test/fixtures/s4_3kph_blocked_scene.json` is extracted read-only from
`logs/kcity_s4_static_obstacle_3kph_safe_01`. It includes reference/pose, footprint,
remembered front/rear anchor points, previous selected direction and bag SHA256.
Tests do not depend on the original bag/cache being present.

The closest recorded scene has front=8.0152235 m, rear=10.6054745 m,
current offset=-0.5703042 m, diagnostic curvature=0.19839485 1/m and best sample
clearance=1.5285867 m. The later user-observed values differ by about 1.4
micrometres in offset and 0.000196 m in clearance; they are not represented as a
byte-identical capture. Reconstructing the old -2.4 m candidate gives curvature
0.19839484 1/m, reproducing 0.198395 to six decimals.

Previously all offsets used `L=max(0.5, front_s-1.0)=7.0152235 m`. The strongest
clearance candidate was -2.4 m, changing offset by -1.829696 m over that distance.
-2.1 m also cleared samples (1.228587 m), but curvature 0.168683 exceeded the
unchanged 0.1604669205 limit. Smaller offsets could be steer-feasible but failed
clearance. Thus no candidate met both gates; the diagnostic was not the maximum
curvature over all offsets.

This bag already had AVOIDING at t=0.985 s, sensor centroid range=15.544 m,
front_s=16.366 m. The old profile stretched entry to 15.366 m. Planning did not
wait until 7.49 m to activate; the profile spread meaningful lateral shift over
almost all available distance. On the first recorded reference the new entry is
about 7.892 m. At s=2.5 m, nominal lateral change increases from 0.060 to 0.338 m.
These are fixed-input path calculations, not a new driven vehicle trajectory.

## Geometry-derived length

For `d(s)=d0+delta*f(t)`, `t=(s-start_s)/L`, retain the quintic:

```text
f(t)  = 10 t^3 - 15 t^4 + 6 t^5, clamped to [0,1]
f'(t) = 30 t^2 (1-t)^2
f''(t)= 60 t (1-t) (1-2t)
kappa(t) = abs(delta)/L^2 * abs(f''(t))
           / (1 + (delta/L*f'(t))^2)^(3/2)
limit = tan(0.70*0.610865)/2.84 = 0.16046692048050848 1/m
```

Solve the symmetric continuous peak over t and bisect length. The upper bracket
`sqrt((10/sqrt(3))*abs(delta)/limit)` follows from max abs(f'')=10/sqrt(3), not an
empirical long-distance constant. `lattice_transition_before_m=5.0` exposes the
existing constructor baseline as a preferred minimum for small shifts. It avoids
generating tiny corrections at maximum curvature and does not override a tighter
front/horizon deadline.

On curved references the analytic length is a starting bound. Evaluate the
actual generated polyline, scan toward the hard deadline and refine a feasible
bracket. BOTH the nominal continuous peak and published-polyline curvature must
pass the unchanged limit. The bounded search may conservatively reject cases it
does not solve; it never relaxes the limit or extends beyond the deadline.
Sampled curvature is not a full dynamics or initial-yaw guarantee.

Entry must finish strictly before `min(obstacle_front_s, reference_horizon)`.
Insert the exact full-offset endpoint in the published polyline: metadata alone
cannot claim completion between samples straddling the front. An entry that
cannot fit is rejected; all failed candidates keep BLOCKED. The default reference
horizon remains 20 m and is never artificially extended.

For the recorded scene, -2.1 m uses entry=7.269177 m, finishing about 0.746046 m
before the front, with clearance=1.228587 m and checked curvature<=0.1604669205.
-2.4 m also fits with entry=7.921340 m. Existing score/direction continuity select
-2.1 m, including the recorded previous -2.1 m direction. A latched BLOCKED state
still requires the existing three safe confirmations before release.

## Hold and recovery

AVOIDING has no embedded return. After entry, hold full offset over the remaining
horizon. Recovery remains forbidden until actual Ego reference progress is
strictly beyond remembered obstacle rear+0.5 m. Bounds persist across shrinking
or missing detections. No guessed vehicle length is introduced.

Alongside the obstacle, an already active same-side actual offset within the
existing 0.15 m threshold of the previous offset can be a zero-shift hold
candidate. It passes the same clearance and curvature gates. This holds the
achieved safe offset instead of inventing a fresh entry after its deadline.
Failed tracking or insufficient actual clearance still causes BLOCKED.

RECOVERING solves the same problem with delta=-current_offset and preferred
`lattice_transition_after_m=10.0`. Increase length if required by curvature. If a
feasible recovery cannot fit in the reference horizon, keep BLOCKED. A shorter
available horizon may be used only when it still passes every gate. Neither a
premature AVOIDING return nor a rear-passage bypass is introduced.

## Stationary RViz diagnostics

- `/planning/lattice_entry_length`: solved/attempted entry length, m.
- `/planning/lattice_full_offset_start_s`: endpoint on reference arc, m.
- `/planning/lattice_recovery_length`: solved/attempted recovery length, m.
- Reuse `/planning/lattice_max_curvature`: maximum used by the gate, including
  nominal continuous and actual polyline checks; no duplicate selected-max topic.
- Existing front/rear, offset, limit, clearance, status, reason and speed cap.

For BLOCKED/RELEASE_PENDING, transition metadata may describe a diagnostic or
proposed candidate, not an approved path. Status/reason and target remain
authoritative. NaN means no applicable transition, e.g. entry during recovery.

Keep `enable_drive=false`, rebuild/restart and read parameters/topics. The
expected current-scene RViz path shifts right from about -0.57 to -2.1 m,
reaches full offset at s about 7.27 m, then stays parallel through rear=10.61 m
and the remaining horizon. FSM can stay READY and target=0 while Lattice displays
AVOIDING; geometric validation does not require arming motion.

This does not implement full-body polygon collision, unknown Sonata overhang or
GPS/rear-axle offsets, drivable-area checks or physical closed-loop validation.
No live MORAI drive was performed. Preserve whole-trial speed limits and verify
actual geometry for a later separately authorized driving test.
