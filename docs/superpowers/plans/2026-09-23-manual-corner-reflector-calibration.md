# Manual Corner Reflector Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Add an operator-controlled browser workflow that freezes matched radar/camera frames, records direct corner-reflector 3D-to-2D samples with mount measurements, and creates independently validated calibration candidates on the PC.

**Architecture:** The browser keeps a versioned, downloadable session and does not command the Pi. A standard-library Python session module validates the file; the existing PC-only OpenCV solver fits only fit samples and evaluates only validation samples.

**Tech Stack:** radar_app.html, Python 3, OpenCV/NumPy on the PC only, unittest, optional Playwright.

## Global Constraints

- Use the 5 cm corner reflector directly; no AprilTag, ChArUco, automatic association, gimbal action, radar configuration change, service restart, or automatic calibration deployment.
- Freeze requires a loaded matched camera frame whose ID equals the matching WebSocket frame ID; later live frames cannot mutate saved samples.
- Each sample preserves raw radar x/y/z, radar frame number, raw index, native camera u/v, camera frame number, set, sync offset, and timestamp.
- Mount input means camera optical centre relative to radar phase centre: right, forward, up are positive. If radar is 10 cm above camera, dz_m is -0.10.
- Runtime translation_m is in camera axes. Calculate camera_center_in_radar_m as -R transpose times t before comparing it to mount input.
- Require at least six fit samples and one validation sample. Only fit samples enter solvePnP.
- Runtime JSON is written only when independent median is at most 8 px and P95 is at most 20 px.
- Local tests and browser simulation are not hardware acceptance. Pi stays free of OpenCV.

---

## File Structure

| File | Responsibility |
| --- | --- |
| tools/fusion/calibration_session.py | Session validation, pair extraction, mount conversion and residual math. |
| tools/fusion/calibrate_radar_camera.py | PC fitting, independent validation, report, gated runtime output. |
| radar_app.html | Frozen selections, mount form, sample list and JSON download. |
| test/test_calibration_session.py | Session/mount unit tests. |
| test/test_calibrate_radar_camera.py | Solver integration tests. |
| test/test_radar_app.py | UI/static/Playwright freeze and pixel tests. |
| docs/radar_camera_calibration.md | Operator procedure. |
| docs/validation/radar-camera-template.md | Calibration evidence fields. |

### Task 1: Define the immutable session contract

**Files:**
- Create: tools/fusion/calibration_session.py
- Create: test/test_calibration_session.py

**Interfaces:**
- Produces SessionError(ValueError) and load_session(path: Path) -> CalibrationSession.
- Produces CalibrationSession.fit_pairs() and validation_pairs(), each returning (x, y, z, u, v).
- Produces camera_center_in_radar(rotation, translation) and mount_residual(measured, solved).

- [ ] **Step 1: Write the failing test**

    def test_session_separates_fit_validation_and_keeps_mount(self):
        session = load_session(write_session(fit_count=6, validation_count=2))
        self.assertEqual(len(session.fit_pairs()), 6)
        self.assertEqual(len(session.validation_pairs()), 2)
        self.assertEqual(session.mount_measurement, (0.0, 0.0, -0.10))

    def test_session_rejects_bad_data(self):
        with self.assertRaisesRegex(SessionError, "at least one validation"):
            load_session(write_session(fit_count=6, validation_count=0))
        with self.assertRaisesRegex(SessionError, "finite"):
            load_session(write_session(mount_dz="bad"))

    def test_camera_centre_conversion(self):
        rotation = ((0, -1, 0), (1, 0, 0), (0, 0, 1))
        self.assertEqual(camera_center_in_radar(rotation, (0.2, -0.1, 0.3)), (0.1, 0.2, -0.3))

- [ ] **Step 2: Run the test to verify it fails**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_calibration_session -v

Expected: ModuleNotFoundError for tools.fusion.calibration_session.

- [ ] **Step 3: Implement the minimum contract**

Create frozen CalibrationSample and CalibrationSession data classes. Require schema version 1, positive image size, six fit samples, one validation sample, finite x/y/z/u/v/sync offset, and raw frame identifiers. Accept no mount data; when present validate finite dx/dy/dz, optional finite yaw/pitch/roll, non-negative uncertainty, and a measurement reference.

Implement:

    def camera_center_in_radar(rotation, translation):
        return tuple(
            -sum(float(rotation[row][column]) * float(translation[row]) for row in range(3))
            for column in range(3)
        )

- [ ] **Step 4: Run the test to verify it passes**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_calibration_session -v

Expected: all session tests pass.

- [ ] **Step 5: Commit**

    git add tools/fusion/calibration_session.py test/test_calibration_session.py
    git commit -m "feat: add calibration session contract"

### Task 2: Build the frozen browser calibration workspace

**Files:**
- Modify: radar_app.html
- Modify: test/test_radar_app.py

**Interfaces:**
- Consumes renderRadarFrame(frame), lastCameraSync, displayed camera-frame metadata and raw radar points.
- Produces calibrationSession, calibrationSnapshot, calibrationRadarSelection, calibrationImageSelection.
- Produces freezeCalibrationSnapshot(), selectCalibrationRadarPoint(point, rawIndex), selectCalibrationImagePixel(event), saveCalibrationSample(), downloadCalibrationSession().

- [ ] **Step 1: Write failing markup and browser tests**

Assert that radar_app.html contains calibrationFreezeBtn, calibrationSampleSet, calibrationSaveSampleBtn, calibrationExportBtn, mountDxInput, mountDyInput, mountDzInput, freezeCalibrationSnapshot, and downloadCalibrationSession.

In a Playwright test inject a matched camera frame and radar frame 88; freeze; select raw point 0; click image centre; save a fit sample; render another live frame. Assert:

    saved = page.evaluate("calibrationSession.samples[0]")
    self.assertEqual(saved["radar"]["frame_num"], 88)
    self.assertEqual(saved["camera"]["u"], 640)
    self.assertEqual(saved["camera"]["v"], 360)
    self.assertEqual(saved["set"], "fit")

- [ ] **Step 2: Run the test to verify it fails**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_radar_app.RadarAppMarkupTests -v

Expected: controls and calibration functions are missing.

- [ ] **Step 3: Add controls and state**

Add a collapsible calibration block in the existing analysis/config panel with freeze, fit/validation selector, save, delete last, export, dx/dy/dz, yaw/pitch/roll, uncertainty, note and status table. Add literal help: dz：相机相对雷达向上为正；雷达高于相机 10 cm 时填 -0.100 m。

Initialize:

    const calibrationSession = {
        schema_version: 1,
        camera_image_size: [cameraCanvas.width, cameraCanvas.height],
        mount_measurement: null,
        samples: [],
    };
    let calibrationSnapshot = null;
    let calibrationRadarSelection = null;
    let calibrationImageSelection = null;

- [ ] **Step 4: Implement freeze and two manual selections**

Reject freeze unless status is matched, displayed camera ID equals sync.frame_id, and a current radar frame exists. Deep-copy raw points and record radar frame number, camera frame ID, image size, offset, capture data and browser timestamp. In calibration mode, PPI selection searches only snapshot raw points; ordinary PPI selection remains unchanged otherwise.

Use native coordinates:

    const rect = cameraCanvas.getBoundingClientRect();
    const u = (event.clientX - rect.left) * cameraCanvas.width / rect.width;
    const v = (event.clientY - rect.top) * cameraCanvas.height / rect.height;
    if (u < 0 || v < 0 || u >= cameraCanvas.width || v >= cameraCanvas.height) return;
    calibrationImageSelection = { u, v };

Draw a crosshair on cameraOverlayCanvas only during calibration mode without changing normal projection behaviour.

- [ ] **Step 5: Save, delete and export**

Reject incomplete selections. Save a JSON row with id, set, radar frame/x/y/z/point_index, camera frame/u/v, offset, timestamp and note. Mount data is null unless dx/dy/dz, uncertainty_m and reference are supplied and valid. The UI must populate uncertainty_m from its numeric field and reference with the literal radar phase centre to camera optical centre when no user note is supplied. Download JSON as radar_camera_calibration_session_<UTC timestamp>.json through a Blob and revoke its URL. Do not send a WebSocket command.

- [ ] **Step 6: Run the focused tests**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_radar_app.RadarAppMarkupTests -v

Expected: static tests pass; Playwright passes or skips only under its existing unavailable-environment condition.

- [ ] **Step 7: Commit**

    git add radar_app.html test/test_radar_app.py
    git commit -m "feat: add manual calibration workspace"

### Task 3: Separate solver fit and independent validation

**Files:**
- Modify: tools/fusion/calibrate_radar_camera.py
- Create: test/test_calibrate_radar_camera.py

**Interfaces:**
- Consumes CalibrationSession and current intrinsics JSON.
- Produces output.report.json after a successful solve.
- Produces runtime calibration JSON only after independent validation passes.

- [ ] **Step 1: Write failing synthetic solver tests**

Create known intrinsics, six perfect fit points, and two separate validation points. Invoke the command via subprocess and assert:

    self.assertEqual(result.returncode, 0)
    report = json.loads(output.with_suffix(".report.json").read_text())
    self.assertEqual(report["fit"]["pair_count"], 6)
    self.assertEqual(report["validation"]["pair_count"], 2)
    self.assertLess(report["validation"]["median_px"], 1e-6)
    self.assertEqual(report["mount_comparison"]["residual_m"], [0.0, 0.0, 0.0])

Add a displaced validation pixel test that exits non-zero, writes no runtime JSON, and reports P95 greater than 20. Add a no-validation test that fails before solve.

- [ ] **Step 2: Run the test to verify it fails**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_calibrate_radar_camera -v

Expected: current command accepts CSV only and calculates one mixed metric.

- [ ] **Step 3: Implement split solving and report**

Load Session from Task 1. Use fit_pairs only for solvePnP. Project validation_pairs separately. Implement:

    def error_metrics(projected, observed):
        errors = np.linalg.norm(projected.reshape(-1, 2) - observed, axis=1)
        return {
            "pair_count": int(len(errors)),
            "rms_px": float(np.sqrt(np.mean(errors ** 2))),
            "median_px": float(np.median(errors)),
            "p95_px": float(np.percentile(errors, 95)),
            "max_px": float(np.max(errors)),
        }

Write fit/validation metrics, camera_center_in_radar_m, mount values and residual to output.report.json. Refuse runtime JSON when independent median is greater than 8.0 or P95 greater than 20.0. Preserve the existing runtime JSON schema used by tools.fusion.calibration.load_calibration.

- [ ] **Step 4: Run solver and compatibility checks**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_calibrate_radar_camera test.test_calibration_session test.test_radar_camera_projection -v

Expected: valid synthetic data writes both files; poor validation writes only report and exits non-zero; existing projection loader remains compatible.

- [ ] **Step 5: Commit**

    git add tools/fusion/calibrate_radar_camera.py tools/fusion/calibration_session.py test/test_calibrate_radar_camera.py test/test_calibration_session.py
    git commit -m "feat: validate manual calibration sessions"

### Task 4: Update runbook and run full regression

**Files:**
- Modify: docs/radar_camera_calibration.md
- Modify: docs/validation/radar-camera-template.md
- Create: test/test_radar_camera_calibration_docs.py

**Interfaces:**
- Consumes the browser session, a separately obtained intrinsics JSON and the PC solver.
- Produces an operator runbook and evidence sheet that distinguish local checks from real hardware acceptance.

- [ ] **Step 1: Write failing documentation test**

    def test_runbook_covers_manual_session_and_independent_gate(self):
        text = Path("docs/radar_camera_calibration.md").read_text(encoding="utf-8")
        for marker in ("冻结匹配帧", "拟合样本", "验证样本", "dz_m = -0.10",
                       "camera_center_in_radar_m", "8 px", "20 px"):
            self.assertIn(marker, text)

- [ ] **Step 2: Run it to verify failure**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_radar_camera_calibration_docs -v

Expected: old CSV/AprilTag procedure lacks manual session and mount guidance.

- [ ] **Step 3: Write the operator procedure**

Document: measure optical-centre to phase-centre dx/dy/dz and uncertainty; verify raw z direction by raising/lowering the reflector; freeze matched frame; select raw point and same image centre; collect 12–20 fit and 6–10 different validation locations; run:

    .\.venv\Scripts\python.exe tools\fusion\calibrate_radar_camera.py <session.json> <intrinsics.json> <output.json> --mount-mode co_rotating

Require output.report.json review and independent median at most 8 px/P95 at most 20 px before manual service configuration. Add template fields for mount values, z-axis test, session hash, fit count, validation count and independent metrics.

- [ ] **Step 4: Run focused and full regression**

Run:

    .\.venv\Scripts\python.exe -m unittest test.test_radar_camera_calibration_docs test.test_calibration_session test.test_calibrate_radar_camera test.test_radar_app -v
    .\.venv\Scripts\python.exe -m unittest discover -s test -v

Expected: all local tests pass; report browser simulation separately from real radar/camera acceptance.

- [ ] **Step 5: Commit and push**

    git add docs/radar_camera_calibration.md docs/validation/radar-camera-template.md test/test_radar_camera_calibration_docs.py
    git commit -m "docs: guide manual radar camera calibration"
    git push origin codex/radar-camera-web-fusion

## Plan Self-Review

- Spec coverage: Tasks 1 through 3 implement immutable sessions, mount measurement comparison, direct manual selections, fit/validation separation and gates; Task 4 covers operator evidence and acceptance.
- Placeholder scan: every step states its concrete test and implementation action.
- Type consistency: browser output matches Task 1 validation; Task 3 consumes Task 1 pairs; solver output keeps the established runtime calibration schema.