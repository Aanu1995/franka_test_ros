# Implement force-ramp grasp analysis in `franka_kitting_controller`

I want you to modify my ROS controller so that grasp evaluation is primarily done by **ramping grip force while the object remains in contact with the gripper** to collect signals for analysis.

## Goal

After contact is detected, the controller should:

1. start at `f_min`
2. apply a grasp at the current `contact_width`
3. wait for the gripper command to complete and for the signal to settle
4. hold at that force for a configurable analysis window
5. update/publish the current ramp-step state
6. update `contact_width` from the latest gripper width after the ramp
7. advance to the next force step (`f += f_step`)
8. repeat until `f_max` is reached
9. then perform the existing `UPLIFT -> EVALUATE` once to determine `SUCCESS` or `FAILURE`

The purpose of the ramp sequence is **data accumulation for later analysis**, especially torque-signal evolution as gripping force increases.

## Required behavior changes

Please remove the old retry mechanism based on:

- bringing the object back down
- increasing force after a failed uplift/evaluate cycle
- repeating grasp attempts through the old retry loop

Please keep:

- `UPLIFT`
- `EVALUATE`
- `SUCCESS`
- `FAILURE`

Please remove:

- `SLIP`
- `DOWNLIFT`
- `SETTLING`
- any repeated retry loop tied to failed evaluation
- any logic that increases force only after a failed evaluation cycle

## Desired state flow

Old style flow was roughly:

`CONTACT -> GRASPING -> UPLIFT -> EVALUATE -> (SLIP / DOWNLIFT / retry / stronger grasp ...)`

I now want:

`CONTACT -> GRASP_1 -> GRASP_2 -> GRASP_3 -> ... -> GRASP_N -> UPLIFT -> EVALUATE -> SUCCESS/FAILURE`

So the force ramp happens first, while maintaining contact with the object.  
After the final ramp step is complete, do one uplift/evaluate pass to classify the grasp outcome.

### Important note about state representation

If it is easiest in code to keep the internal enum as a base grasp state, that is okay, but the **published/logged state must clearly identify the current ramp step**.

For example:

- `GRASP_1`
- `GRASP_2`
- `GRASP_3`
- `GRASP_4`

Also update `grasp_iteration` or equivalent ramp index in `KittingState` for every ramp.

## Minimal implementation preference

Please keep the implementation as **minimal and localized** as possible.

Do **not** rewrite the whole controller.

This should replace the old post-contact retry behavior with a contact-only force-ramp sequence followed by a single final uplift/evaluate pass.

## Core implementation requirements

### 1. Keep only one final uplift/evaluate decision

After contact is detected and all grasp-force ramp steps are completed:

- transition to `UPLIFT`
- then transition to `EVALUATE`
- from there, go directly to:
  - `SUCCESS`, or
  - `FAILURE`

Do **not** re-enter a retry cycle.

Do **not** perform:

- downlift and retry
- force increase after failed evaluation
- repeated grasp/evaluate loops

### 2. Add/configure ramp parameters

Add parameters similar to:

- `grasp_force_min`
- `grasp_force_max`
- `grasp_force_step`
- `grasp_force_hold_time`
- `grasp_settle_time`
- `grasp_width_epsilon`

Use existing naming/style conventions from the package if there is already a better match.

### 3. Force ramp sequence before uplift

After contact detection:

- initialize current ramp force to `grasp_force_min`
- command grasp using:
  - width = current `contact_width`
  - force = current ramp force
  - epsilon = existing configured epsilon / grasp epsilon behavior
- wait until the gripper action completes
- wait `grasp_settle_time`
- hold/log for `grasp_force_hold_time`
- publish/log state as `GRASP_<step>`
- record/store useful data for that step if logging hooks already exist
- update `contact_width`
- increment force by `grasp_force_step`
- continue until current force reaches `grasp_force_max`
- then transition to `UPLIFT`

If a grasp command fails, times out, or another existing failure condition is hit, transition to `FAILURE`.

## Contact width update requirement

After **each ramp**, update `contact_width` from the latest measured/current gripper width.

However, **maintain the configured epsilon behavior**.  
Do not remove or ignore the epsilon already used around grasp width.  
In other words:

- refresh `contact_width` after each completed ramp step
- continue commanding grasp widths using that updated contact width
- preserve the specified epsilon around the width target

This is important because I want the width target to track the real contact condition across ramp steps without losing the tolerance behavior.

## Suggested state variables

Please add or reuse minimal state needed for the ramp flow, such as:

- `double current_grasp_force_`
- `int grasp_iteration_`
- `ros::Time grasp_step_start_time_`
- `bool grasp_command_sent_`
- `bool grasp_step_complete_`

Only add what is necessary.

## Suggested controller logic

A clean minimal approach would be:

### On contact detection

- set `grasp_iteration_ = 1`
- set `current_grasp_force_ = grasp_force_min`
- set `contact_width_` from current measured width if needed
- transition to grasp ramp state

### In grasp ramp handling

For the current ramp step:

1. send grasp command with:
   - target width = `contact_width_`
   - epsilon = configured epsilon
   - force = `current_grasp_force_`

2. wait for command completion

3. wait for settle time

4. hold for analysis window

5. publish/update state label:
   - `GRASP_<grasp_iteration_>`

6. update `contact_width_` from measured gripper width

7. if `current_grasp_force_ + grasp_force_step <= grasp_force_max`
   - `current_grasp_force_ += grasp_force_step`
   - `grasp_iteration_ += 1`
   - start next ramp step
     else
   - transition to `UPLIFT`

### In uplift/evaluate handling

Keep the existing uplift/evaluate implementation as much as possible, but simplify the transitions:

- `UPLIFT -> EVALUATE -> SUCCESS`
- `UPLIFT -> EVALUATE -> FAILURE`

No retry branch after `FAILURE`.

No slip/downlift loop.

## Logging / publishing expectations

Please make sure the following are visible in logs or state messages:

- current grasp ramp index
- current commanded grasp force
- current contact width used for the command
- updated measured contact width after the step
- published state label as `GRASP_<N>`
- final result from `EVALUATE`

If `KittingState` already exposes relevant fields, reuse them.  
If one small extension is needed, keep it minimal.

## Important constraints

- keep implementation localized to `franka_kitting_controller`
- keep `UPLIFT` and `EVALUATE` for the final outcome decision
- remove retry behavior after failed evaluation
- remove `SLIP`, `DOWNLIFT`, and `SETTLING`
- do not reintroduce repeated re-grasp loops
- preserve `SUCCESS` / `FAILURE`
- preserve epsilon handling for grasp width
- update `contact_width` after every ramp
- published state should distinguish each ramp step
