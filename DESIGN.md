# Design

## Source of truth
Active, 2026-10-07. Desktop Qt Widgets, one window. References: the assignment PDF,
README.md, src/mainwindow.ui, and user requests for circular motion, live filter
correction, the supplied drone SVG, and flight audio.

## Product goals
Help a student compare true motion, inertial drift, and complementary filtering.
Show the effect of disabling and restoring external measurements during motion.
This is an educational simulation, not a flight controller.

## Layout and appearance
Place the trajectory scene on the left and scrollable controls on the right.
Use a light background, an adaptive coordinate grid, and stronger zero axes.
Truth is green, raw INS is blue, and the filtered estimate is orange.
Label trajectories in text as well as color. Use system fonts.
Keep the scene at least 500 logical pixels wide and preserve manual camera settings.

## Motion and correction
Configure constant acceleration or a clockwise circle before starting.
Lock mode, radius, and period until reset. Allow acceleration edits while paused.
Keep automatic correction, sensor availability, noise, and response time editable
during motion and pause. Switching correction must preserve position, velocity,
simulation time, and trajectory history. Apply feedback only on fresh measurements.
The assignment's one-shot correction acts on raw INS only and resets its velocity.
Reset clears all trajectories and repeats the noise sequence while retaining settings.

## Drone and audio
Anchor the provided SVG at the true position and orient it along true velocity.
Keep its displayed size independent of scene zoom.
Embed SVG and MP3 assets in Qt resources rather than using external absolute paths.
Flight sound is opt-in at 30% volume, loops during motion, pauses with the simulation,
and rewinds on reset. Switching sound must not affect motion or correction.
Report playback errors without interrupting the simulator.

## Accessibility and controls
Use Russian UI labels and tooltips with metres, seconds, and metres per second squared.
Keyboard controls: +/- for zoom, arrows for pan, and F for automatic overview.
Manual camera input disables automatic overview; the checkbox or F restores it.
Keep marker labels readable at every zoom level and controls accessible by keyboard.

## Implementation and verification
C++17, Qt Widgets, Svg, and Multimedia; Qt 6 also requires SvgWidgets.
Keep the numerical model independent of Qt and the controls editable in Designer.
Verify motion, correction toggles, sensor loss, and reset with numerical tests.
Verify GUI controls, drone anchoring, camera behavior, and audio lifecycle with QtTest.
Inspect screenshots on the native Windows platform after layout changes.
Document the tested toolchain and do not imply untested Qt/MSVC combinations passed.
