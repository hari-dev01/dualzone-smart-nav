DualZone Smart Assistive Navigation — Clean Wokwi Rebuild

This replaces the previous single-node integrated diagram/sketch with a simpler, consistently wired test bench. It preserves the mdptrial-style state logic while removing three unnecessary potentiometers and excess buzzer channels.

Files





sketch.ino — complete Arduino-ESP32 sketch.



diagram.json — Wokwi layout with three visual sections: cane/test inputs, ESP32, and belt sensors/indicators.



PIN_MAP.md — component-to-pin wiring table and electrical notes.



REFERENCE_NOTES.md — official component/API references used for the design.



VALIDATION_REPORT.md — checks run and limitations.



validate_project.py — static project validator.



build_diagram.py — portable script used to generate diagram.json in the same folder.



tests/decision_logic_test.cpp — host-side smoke tests for the hazard decision logic.



stubs/ — minimal Arduino/Wire API stubs used only for host-side validation; not firmware libraries.

Design choices

Three potentiometers only





Cane range simulation (5–80 cm proxy), GPIO 34.



Cane pitch simulation (−60°…+60°), GPIO 35.



Detected-object lateral offset (−2.5…+2.5 m proxy), GPIO 39.

The front HC-SR04 provides the simulated object's range. Its measured change over time provides a heuristic closing-speed estimate. The confidence score is based on sample validity and measurement jitter; it is not read from a dedicated confidence potentiometer.

Seven labelled LEDs, three directional hazard sounders

The 74HC595 controls the seven one-hot visual states through seven separate 330 Ω LED resistors. Three independent GPIO sounders map to LEFT, FRONT, and RIGHT hazards. Audio is deliberately stricter than the display: a buzzer is allowed only after a high-confidence, three-sample confirmed path conflict or a stable, three-sample drop-off proxy. Simple near-distance indications, RAISED / UNEVEN, SYSTEM CLEAR, and LOW CONFIDENCE are visual-only and remain silent. The drop-off proxy uses the front buzzer with an urgent triple-pulse pattern; confirmed directional conflicts use the matching zone buzzer. Audio is never electrically in series with an LED branch.

This reduces unnecessary sound hardware and avoids putting buzzer current through the 74HC595. In the Wokwi diagram the supported piezo buzzer models connect to their own GPIO and GND; physical high-current sounders need their own driver stages as explained in PIN_MAP.md.

Noise-resistant state engine





One sonar is triggered per scheduled scan slot; no two ultrasonic sensors are pinged together.



Each echo wait has a bounded timeout.



Each zone keeps a rolling 3-sample history and EMA-filtered distance.



Closing speed uses measured elapsed time; closing > 0 means distance is decreasing.



One missed echo cannot be interpreted as a confirmed clear path.



Raised/uneven and drop-off proxy conditions need three consecutive fresh potentiometer samples.



Path conflict requires matching lateral-sector alignment, valid low-noise readings, confidence ≥0.60, positive measured closing speed above 0.08 m/s, TTC ≤3 s, and three consecutive samples from the relevant zone.



Ordinary indicator transitions are debounced using fresh observations; critical states appear after their source condition is confirmed.



Tone patterns run non-blockingly and only one audio channel is active at once.



Serial input is parsed as complete lines, so a multi-character command sequence does not trigger repeated recalibration per character.

Run in Wokwi





Open a new ESP32 DevKit C V4 Wokwi project.



Replace diagram.json and sketch.ino with these files.



Start simulation and open the Serial Monitor at 115200 baud.



On startup, the firmware non-blockingly calibrates the cane baseline over 20 samples. Keep the range and pitch potentiometers at their chosen normal-floor test values during this short period.



Type ? and press Enter for the JSON-formatted logic notes.



Type c and press Enter to recalibrate; keep range/pitch at the normal-floor test values while it samples.

How to exercise states





SYSTEM CLEAR: all three sensors valid and stable, no confirmed near zone, no raised/drop-off/path-conflict condition.



RAISED / UNEVEN: after calibration, reduce the cane range proxy enough to make projected height fall at least 10 cm below its baseline for three fresh samples.



DROP-OFF proxy: increase the cane range proxy enough to make projected height rise more than 15 cm above baseline for three fresh samples. This is not a real drop-off measurement.



LEFT / FRONT / RIGHT: use the matching HC-SR04 slider and hold the zone within approximately 100 cm for three valid samples. The nearest confirmed zone wins.



PATH CONFLICT heuristic: move the lateral-offset potentiometer into the matching sector (negative for LEFT, near zero for FRONT, positive for RIGHT). Then move that zone's distance slider from farther to closer so the system measures positive closing speed. Sensor confidence must recover above threshold, TTC must remain within 3 seconds, and three fresh samples must confirm the candidate before audio is enabled.



LOW CONFIDENCE: create a sensor timeout/invalid condition or wait for an input to become stale (where the simulator permits that condition).

The path-conflict event requires a measured change in the corresponding zone range; a static close object alone must not produce a TTC-based conflict or sound an alarm. Side conflicts also require the lateral-offset proxy to match the same side as the detecting sensor.

Serial format

After startup events, recurring telemetry is one JSON object per line with ts_ms, cane_range_cm_proxy, projected_h_cm, delta_h_cm, per-zone ranges, front_closing_mps_heuristic, ttc_s_heuristic, confidence fields, confirmation counts, current indicator, and currently selected buzzer information. null means the estimate is unavailable.

Limitations





This is a one-board Wokwi test harness; it does not implement the true BLE connection between separate cane and belt boards.



Potentiometers are test proxies, not real ToF/IMU measurements.



Projected-height change is not definitive drop-off detection. It depends on baseline and assumed geometry.



TTC, confidence and path conflict are experimental heuristics, not validated safety metrics.



Each HC-SR04 provides only sector range and a rough range-change closing-speed heuristic; lateral alignment comes from the third manual simulation control. A confirmed sector conflict requires a matching left/front/right lateral region, valid and low-noise readings, positive closing speed, TTC within the configured horizon, and three consecutive valid samples.



Wokwi's board and sound model do not validate physical electrical current or real acoustic/hardware behaviour.



The design is an educational/research prototype, not a certified mobility aid.

Validation

Run the static wiring checker and host-side decision smoke test:

python validate_project.py
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -I stubs tests/decision_logic_test.cpp stubs/globals.cpp -o decision_logic_test
./decision_logic_test

The script checks JSON syntax, unique IDs, component counts, all connections, per-LED resistor paths, separate buzzer circuits, pin uniqueness, and the sketch's expected pin constants. The sketch was syntax-checked with g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -x c++ -I stubs -fsyntax-only sketch.ino against minimal Arduino/Wire stubs. These host checks are not a replacement for compiling under the actual Arduino-ESP32 core or running Wokwi.
