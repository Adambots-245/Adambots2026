# Tools

Team utility tools for planning, analysis, and offline work. These are **not deployed to the robot** — they're dev-side helpers.

## Files

### `power_budget.html`

Standalone HTML tool for modeling the robot's current draw and battery voltage sag. Single-file, opens in any browser — no build step, no dependencies.

**What it does:**
- Maintains a motor inventory (pre-seeded from `Constants.java` + `RobotMap.java` + swerve JSON)
- Calculates per-scenario total supply current, peak current, and predicted voltage sag
- Includes per-mechanism calculators (flywheel, arm, drivetrain, elevator, spindexer, uptake) that derive current from physical load — torque, force, or energy — through gear ratio + motor curves
- Editable simultaneity matrix (scenarios × functions) for budget sanity
- Unit converter popup (in²·lb ↔ kg·m², m/s² ↔ ft/s², lbf ↔ N, etc.)

**How to use:**
1. Open `tools/power_budget.html` in a browser (double-click works)
2. Budget tab — pick a scenario, watch the voltage column
3. Methodology tab — drill into any mechanism to size its current limits

**Keeping it in sync with the code:**
- When a current limit in `Constants.java` changes, update the matching row in the tool's `DEFAULTS` array (search for the constant name in the notes column)
- When a new motor or mechanism is added, add a row to `DEFAULTS` and — if applicable — to the simultaneity matrix scenarios
- The tool intentionally does not parse `Constants.java` at runtime; updates are manual so the numbers match the *intent* at a point in time, not whatever's uncommitted on a branch

Source of truth: the Java constants. The tool mirrors them for planning purposes.
