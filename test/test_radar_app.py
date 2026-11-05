import base64
import json
import struct
import unittest
import zlib
from pathlib import Path

from tools.fusion.calibration_session import load_session


try:
    from playwright.sync_api import sync_playwright
except ImportError:  # pragma: no cover - development environment may not bundle Playwright
    sync_playwright = None


CHROME_EXECUTABLE = Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe")
CAMERA_IMAGE = base64.b64decode(
    "R0lGODlhAQABAIAAAAAAAP///ywAAAAAAQABAAACAUwAOw=="
)


def camera_png(width, height):
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    rows = (b"\x00" + b"\x30\x60\x90" * width) * height
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">2I5B", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


class RadarAppMarkupTests(unittest.TestCase):
    def test_intensity_ppi_offers_background_subtraction_with_raw_comparison(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('id="ppiContrastMode"', app_html)
        self.assertIn('value="adaptive" selected', app_html)
        self.assertIn('value="raw"', app_html)

    def test_live_intensity_ppi_has_explicit_view_switch_and_caveat(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('id="ppiViewSelect"', app_html)
        self.assertIn('value="intensity"', app_html)
        self.assertIn('未采样方位留白', app_html)
        self.assertIn('未经实测标定', app_html)

    def test_motor_frame_estimates_are_next_to_rpm_and_match_100_ms_cfg(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        panel = app_html.split('id="gimbalScanPanel"', 1)[1].split('<div class="main-visual">', 1)[0]
        self.assertLess(panel.index('id="motorRpmInput"'), panel.index('id="motorFramePeriodInput"'))
        self.assertLess(panel.index('id="motorFramePeriodInput"'), panel.index('id="motorFrameAngleDisplay"'))
        self.assertLess(panel.index('id="motorFrameAngleDisplay"'), panel.index('id="motorAngleDisplay"'))
        self.assertIn('id="motorRpmInput" value="30"', panel)
        self.assertIn('id="motorFramePeriodInput" value="0.1"', panel)
        self.assertIn('id="motorFrameAngleDisplay" value="18.00"', panel)

    def test_qd4310_panel_exposes_speed_and_stop_without_pwm_controls(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('id="motorRpmInput"', app_html)
        self.assertIn('id="gimbalScanToggleBtn"', app_html)
        self.assertIn('id="motorApplyRpmBtn"', app_html)
        self.assertIn('id="motorDirectionSelect"', app_html)
        self.assertIn('id="motorAngleDisplay"', app_html)
        self.assertIn('电机单圈角度', app_html)
        self.assertNotIn('id="motorPwmDutyInput"', app_html)
        self.assertIn("mode: 'qd4310'", app_html)

    def test_live_registration_is_opt_in_and_visualises_only_measured_points(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        self.assertIn('id="liveRegistrationEnabled"', app_html)
        self.assertIn("live_registration_enabled", app_html)
        self.assertIn("liveRegistration.state === 'provisional'", app_html)
        self.assertIn("renderGimbalScanFrame(data)", app_html)

    def test_manual_calibration_workspace_declares_local_controls(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        for marker in (
            'id="calibrationFreezeBtn"',
            'id="calibrationSampleSet"',
            'id="calibrationSaveSampleBtn"',
            'id="calibrationExportBtn"',
            'id="mountDxInput"',
            'id="mountDyInput"',
            'id="mountDzInput"',
            'id="mountUncertaintyInput"',
            "freezeCalibrationSnapshot",
            "downloadCalibrationSession",
            "radar phase centre to camera optical centre",
            "直接使用 5 cm 角反射器",
        ):
            self.assertIn(marker, app_html)

    def test_camera_panel_declares_bounded_synchronised_frame_contract(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )

        for marker in (
            'id="cameraCanvas"',
            'id="cameraStatus"',
            'id="cameraFrameId"',
            'id="cameraSyncOffset"',
            'id="cameraDisplayMode"',
            'id="cameraOverlayCanvas"',
            "fetchCameraFrame",
            "drawCameraFrame",
            "cache: 'no-store'",
            "createImageBitmap(blob)",
            "frame.bitmap.close()",
            "AbortController",
            "updateCameraPanel(data.camera_sync)",
            "软件接收时钟同步，非硬件触发同步",
            "尚未空间标定",
        ):
            self.assertIn(marker, app_html)
        self.assertNotIn(".mjpeg", app_html.lower())

    def test_camera_overlay_controls_fail_closed_and_explain_projection_limits(self):
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )
        for marker in (
            'id="cameraOverlayEnabled"',
            'id="cameraOverlayLabels"',
            'id="cameraCalibrationStatus"',
            'id="cameraProjectionReason"',
            'id="cameraOverlayOpacity"',
            'id="cameraOverlayDebug"',
            "drawCameraProjection",
            "投影点是坐标配准结果，不代表目标已被分类或确认。",
        ):
            self.assertIn(marker, app_html)

    def test_control_sidebar_declares_independent_scroll_contract(self):
        """The operator controls must remain reachable on short displays."""
        app_html = (Path(__file__).resolve().parents[1] / "radar_app.html").read_text(
            encoding="utf-8"
        )

        self.assertIn(".hud-sidebar", app_html)
        self.assertIn("max-height: calc(100vh - 124px)", app_html)
        self.assertIn("overflow-y: auto", app_html)
        self.assertIn(".hud-sidebar::-webkit-scrollbar-thumb", app_html)
        self.assertIn(".hud-overlay.hud-sidebar", app_html)
        self.assertIn("pointer-events: auto", app_html)
        self.assertNotIn(".status-panel button:not(#connectBtn)", app_html)
        for selector in (
            'id="analysisContent"',
            'id="replayAnalysisSummary"',
            'id="replayTrendCanvas"',
            'id="replayFrameDetail"',
            'id="replayTargetList"',
            'id="replayPointDetail"',
            "requestReplayAnalysis",
            "replay_analysis",
            "clusterReplayPoints",
            "updateReplayClusterTracks",
            "describeReplayClusterShape",
            "nearestRangeM",
            "edge/shoreline",
            "near ${cluster.nearestRangeM.toFixed(2)} m",
            "REPLAY_STABLE_CLUSTER_MIN_HITS",
            ".analysis-target-row.candidate",
            "const stateClass = cluster.isStable ? '' : ' candidate'",
            "scrollIntoView({ block: 'nearest' })",
            "if (position <= previousPosition) resetReplayClusterTracks()",
            "handleReplayCanvasClick",
            'id="blindZoneInput"',
            "RADAR_BLIND_ZONE_STORAGE_KEY",
            "getBlindZoneM",
            "loadBlindZoneParam",
            "handleLiveCanvasClick",
            "renderLivePointDetail",
            "nearest raw",
            "nearest shown",
            'id="liveObjectToggleBtn"',
            "LIVE_OBJECT_ENABLED_STORAGE_KEY",
            "liveObjectEnabled",
            "实时物体显示已关闭",
            "drawReplayClusterOverlay(clusters, selectedClusterId, selectedPoint, labelPrefix = 'T')",
            "drawReplayClusterOverlay(clusters, liveSpatialAnalysis.selectedClusterId, null, 'L')",
            'id="nearestObstacleCard"',
            "DOCKING OBSTACLE VIEW",
            "NEAREST STABLE OBSTACLE",
            "采集质量 / 调试信息",
            "renderNearestObstacleCard",
            "nearestStable",
            "LIVE FRONT OBJECT",
            "LIVE_CLUSTER_RADIUS_M",
            "liveForwardClusters",
            "updateLiveClusterTracks",
            "renderLiveForwardObjectList",
            "drawLiveClusterOverlay",
            "collisionHint",
            "实时前方物体",
            "LIVE_CLUSTER_SMOOTHING",
            "LIVE_PANEL_UPDATE_INTERVAL_MS",
            "lastLivePanelRenderAt",
            "lastLiveDetailRenderAt",
            ".analysis-target-list",
            "height: clamp(168px, 22vh, 210px)",
            "align-content: start",
            ".analysis-point-detail",
            "height: clamp(112px, 16vh, 180px)",
            "overscroll-behavior: contain",
            'data-filter-tab="signal"',
            'data-filter-tab="near"',
            'data-filter-tab="line"',
            'id="filterPanelNear"',
            "switchFilterTab",
            'id="f_azimMin"',
            'id="f_azimMax"',
            'id="f_elevMin"',
            'id="f_elevMax"',
            "aoaFovCfg -1 ${azimMin} ${azimMax} ${elevMin} ${elevMax}",
            'id="localMapBadge"',
            "LOCAL MAP FRAMEWORK",
            "transformRadarPointToLocalMap",
            "currentLocalMapPose",
            "poseCompensated",
            "未接入位姿补偿",
            "updateLocalMapBadge",
            "局部地图: 开",
            "局部地图: 关",
        ):
            self.assertIn(selector, app_html)
        self.assertNotIn('id="f_fovAngle"', app_html)
        self.assertNotIn("return `aoaFovCfg -1 -${ang} ${ang} -${ang} ${ang}`", app_html)


@unittest.skipUnless(
    sync_playwright is not None and CHROME_EXECUTABLE.is_file(),
    "Playwright or the local Chrome executable is not installed",
)
class RadarAppTests(unittest.TestCase):
    def _new_browser(self, playwright):
        return playwright.chromium.launch(headless=True, executable_path=str(CHROME_EXECUTABLE))

    def test_intensity_ppi_keeps_four_hundred_measured_spokes(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            observed = page.evaluate("""() => {
                const spokes = Array.from({length: 400}, (_, index) => ({
                    seq: index + 1, yaw_deg: index * 0.8,
                    range_profile: [100, 200, 300], range_profile_bin_size_m: 0.05,
                }));
                ingestIntensityPpi({generation: 'test-400', oldest_seq: 1, spokes});
                return [intensityPpiState.spokes.length,
                    intensityPpiState.spokes[0].seq,
                    intensityPpiState.spokes.at(-1).seq];
            }""")
            self.assertEqual(observed, [400, 1, 400])
            browser.close()

    def test_adaptive_ppi_contrast_separates_local_echo_from_range_pedestal(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const spokes = Array.from({length: 32}, (_, angle) => ({
                    range_profile: Array.from({length: 32}, (_, bin) =>
                        500 + bin * 20 + (angle % 3) * 2),
                }));
                spokes[5].range_profile[10] += 300;
                const enhanced = buildIntensityPpiContrastModel(spokes, 1, 31, 1, true);
                const raw = buildIntensityPpiContrastModel(spokes, 1, 31, 1, false);
                return {
                    target: ppiIntensityNormalized(spokes[5].range_profile[10], 10, enhanced),
                    background: ppiIntensityNormalized(spokes[6].range_profile[10], 10, enhanced),
                    rawBackground: ppiIntensityNormalized(spokes[6].range_profile[10], 10, raw),
                    original: spokes[5].range_profile[10],
                };
            }""")
            self.assertGreater(result["target"], 0.8, result)
            self.assertLess(result["background"], 0.1, result)
            self.assertGreater(result["rawBackground"], result["background"] + 0.15, result)
            self.assertEqual(result["original"], 1004)
            browser.close()

    def test_ppi_issue1_live_intensity_respects_metre_blind_zone(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                document.getElementById('ppiViewSelect').value = 'intensity';
                document.getElementById('f_fovRange').value = '3.5';
                document.getElementById('blindZoneInput').value = '0.5';
                const profile = Array(100).fill(100);
                for (let i = 2; i <= 7; i++) profile[i] = 900;
                for (let i = 52; i <= 61; i++) profile[i] = 900;
                const frame = (spokes) => ({frame_num: 1, points: [],
                    range_profile: profile, range_profile_bin_size_m: 0.05,
                    gimbal_scan: {enabled: true, mode: 'qd4310', points: [],
                        live_registration: {state: 'provisional'}, intensity_ppi: {
                            generation: 'test', oldest_seq: 1, latest_seq: 1, spokes}}});
                const green = (x) => {
                    const data = ctx.getImageData(x - 2, 398, 5, 5).data;
                    let sum = 0;
                    for (let i = 1; i < data.length; i += 4) sum += data[i];
                    return sum;
                };
                renderRadarFrame(frame([]));
                const nearBefore = green(420), farBefore = green(689);
                renderRadarFrame(frame([{seq: 1, yaw_deg: 90,
                    range_profile: profile, range_profile_bin_size_m: 0.05}]));
                return {nearGain: green(420) - nearBefore,
                    farGain: green(689) - farBefore};
            }""")
            self.assertLess(abs(result["nearGain"]), 50, result)
            self.assertGreater(result["farGain"], 50, result)
            browser.close()

    def test_ppi_issue2_display_threshold_and_brightness_are_adjustable(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const threshold = document.getElementById('ppiDisplayThresholdInput');
                const brightness = document.getElementById('ppiDisplayBrightnessInput');
                if (!threshold || !brightness) return {missingControls: true};
                document.getElementById('ppiViewSelect').value = 'intensity';
                document.getElementById('f_fovRange').value = '3.5';
                const profile = Array(100).fill(100);
                for (let i = 52; i <= 61; i++) profile[i] = 900;
                const frame = {frame_num: 1, points: [], range_profile: profile,
                    range_profile_bin_size_m: 0.05, gimbal_scan: {enabled: true,
                        mode: 'qd4310', points: [], live_registration: {state: 'provisional'},
                        intensity_ppi: {generation: 'test', oldest_seq: 1, latest_seq: 1,
                            spokes: [{seq: 1, yaw_deg: 90, range_profile: profile,
                                range_profile_bin_size_m: 0.05}]}}};
                const green = () => {
                    const data = ctx.getImageData(687, 398, 5, 5).data;
                    let sum = 0;
                    for (let i = 1; i < data.length; i += 4) sum += data[i];
                    return sum;
                };
                threshold.value = '0'; brightness.value = '100';
                renderRadarFrame(frame);
                const bright = green();
                threshold.value = '90'; brightness.value = '25';
                renderIntensityPpiFrame(frame, true);
                return {missingControls: false, bright, dimmed: green(),
                    rawUnchanged: profile[55] === 900};
            }""")
            self.assertFalse(result["missingControls"], result)
            self.assertTrue(result["rawUnchanged"], result)
            self.assertLess(result["dimmed"], result["bright"] - 100, result)
            browser.close()

    def test_ppi_issue3_controls_do_not_cover_radar_canvas(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const toolbar = document.querySelector('.ppi-view-toolbar').getBoundingClientRect();
                const radar = document.getElementById('radarCanvas').getBoundingClientRect();
                return {toolbarBottom: toolbar.bottom, canvasTop: radar.top,
                    overlaps: toolbar.left < radar.right && toolbar.right > radar.left &&
                        toolbar.top < radar.bottom && toolbar.bottom > radar.top};
            }""")
            self.assertFalse(result["overlaps"], result)
            browser.close()

    def test_live_intensity_ppi_keeps_measured_spokes_and_clears_on_reset(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                document.getElementById('ppiViewSelect').value = 'intensity';
                const spoke = (seq, yaw) => ({seq, frame_num: seq,
                    yaw_deg: yaw, range_profile: [100, 100, 900, 100],
                    range_profile_bin_size_m: 0.05, rf_time_calibrated: false});
                const frame = (number, batch) => ({frame_num: number, points: [],
                    range_profile: [100, 100, 900, 100], range_profile_bin_size_m: 0.05,
                    gimbal_scan: {enabled: true, mode: 'qd4310', status: 'running',
                        points: [], live_registration: {state: 'provisional'},
                        intensity_ppi: batch}});
                renderRadarFrame(frame(1, {generation: 'a', oldest_seq: 1,
                    latest_seq: 1, spokes: [spoke(1, 15)]}));
                renderRadarFrame(frame(2, {generation: 'a', oldest_seq: 1,
                    latest_seq: 2, spokes: [spoke(2, 90)]}));
                const observed = intensityPpiState.spokes.map(item => item.yaw_deg);
                const label = document.getElementById('intensityPpiStatus').textContent;
                renderRadarFrame(frame(3, {generation: 'b', oldest_seq: 1,
                    latest_seq: 0, spokes: []}));
                return {observed, label, afterReset: intensityPpiState.spokes.length};
            }""")
            self.assertEqual(result["observed"], [15, 90])
            self.assertIn("2", result["label"])
            self.assertEqual(result["afterReset"], 0)
            browser.close()

    def test_live_intensity_ppi_draws_observed_ray_without_painting_opposite_side(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                document.getElementById('ppiViewSelect').value = 'intensity';
                document.getElementById('f_fovRange').value = '3.5';
                const profile = Array(100).fill(100);
                for (let i = 52; i <= 61; i++) profile[i] = 900;
                const frame = (number, spokes) => ({frame_num: number, points: [],
                    range_profile: profile, range_profile_bin_size_m: 0.05,
                    gimbal_scan: {enabled: true, mode: 'qd4310', status: 'running', points: [],
                        live_registration: {state: 'provisional'}, intensity_ppi: {
                            generation: 'measured', oldest_seq: 1, latest_seq: number, spokes}}});
                const green = (x, y) => {
                    const data = ctx.getImageData(x - 2, y - 2, 5, 5).data;
                    let sum = 0;
                    for (let i = 1; i < data.length; i += 4) sum += data[i];
                    return sum;
                };
                renderRadarFrame(frame(1, []));
                const beforeRight = green(689, 400);
                const beforeLeft = green(111, 400);
                renderRadarFrame(frame(2, [{seq: 1, yaw_deg: 90,
                    range_profile: profile, range_profile_bin_size_m: 0.05}]));
                const rightGain = green(689, 400) - beforeRight;
                const leftGain = green(111, 400) - beforeLeft;
                const reset = frame(3, []);
                reset.gimbal_scan.intensity_ppi.generation = 'new-sweep';
                renderRadarFrame(reset);
                return {rightGain, leftGain,
                    resetDifference: green(689, 400) - beforeRight};
            }""")
            self.assertGreater(result["rightGain"], 50, result)
            self.assertGreater(result["rightGain"], abs(result["leftGain"]) + 50, result)
            self.assertLess(abs(result["resetDifference"]), 10, result)
            browser.close()

    def test_range_profile_axis_uses_cfg_bin_size_not_ppi_display_range(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            labels = page.evaluate("""() => {
                const labels = [];
                const original = rangeProfileCtx.fillText;
                rangeProfileCtx.fillText = function(text, ...args) { labels.push(String(text)); };
                try {
                    document.getElementById('f_fovRange').value = '3.5';
                    const profile = Array(512).fill(1);
                    profile[73] = 100;
                    renderRadarFrame({frame_num: 1, points: [], range_profile: profile,
                        range_profile_bin_size_m: 0.027915245, camera_sync: null});
                    return labels;
                } finally { rangeProfileCtx.fillText = original; }
            }""")
            self.assertIn("Peak 2.04 m", labels)
            self.assertIn("14.3 m", labels)
            self.assertNotIn("3.5 m", labels)
            browser.close()

    def test_range_profile_without_active_cfg_shows_bins_not_false_metres(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            labels = page.evaluate("""() => {
                const labels = [];
                const original = rangeProfileCtx.fillText;
                rangeProfileCtx.fillText = function(text, ...args) { labels.push(String(text)); };
                try {
                    document.getElementById('f_fovRange').value = '3.5';
                    const profile = Array(512).fill(1);
                    profile[73] = 100;
                    renderRadarFrame({frame_num: 1, points: [], range_profile: profile,
                        camera_sync: null});
                    return labels;
                } finally { rangeProfileCtx.fillText = original; }
            }""")
            self.assertIn("Peak bin 73", labels)
            self.assertIn("Range bin (uncalibrated)", labels)
            self.assertNotIn("3.5 m", labels)
            browser.close()

    def test_snr_display_gate_is_off_by_default_and_counts_raw_vs_shown(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            frame = {
                "frame_num": 1,
                "points": [
                    {"x": 0.1, "y": 2.2, "z": 0, "v": 0, "snr": -1.1, "noise": 78.4},
                    {"x": 0.2, "y": 2.3, "z": 0, "v": 0, "snr": 12.0, "noise": 78.4},
                ],
                "range_profile": [],
                "gimbal_scan": {"enabled": False, "status": "idle", "points": []},
            }
            page.evaluate("frame => renderRadarFrame(frame)", frame)
            self.assertEqual(page.locator("#minSnrSelect").input_value(), "off")
            self.assertIn("2/2", page.locator("#pointsDisplay").inner_text())
            self.assertIn("原始/显示", page.locator("#pointsDisplay").get_attribute("title"))
            page.locator("#minSnrSelect").select_option("10")
            page.evaluate("frame => renderRadarFrame(frame)", frame)
            self.assertIn("2/1", page.locator("#pointsDisplay").inner_text())
            page.locator("#minSnrSelect").select_option("custom")
            page.locator("#minSnrCustomInput").fill("-2")
            page.evaluate("frame => renderRadarFrame(frame)", frame)
            self.assertIn("2/2", page.locator("#pointsDisplay").inner_text())
            page.locator("#minSnrSelect").select_option("off")
            page.evaluate("frame => renderRadarFrame(frame)", frame)
            self.assertIn("2/2", page.locator("#pointsDisplay").inner_text())
            page.evaluate("localStorage.setItem('radar_noise_filter_enabled', 'true')")
            page.reload(wait_until="networkidle")
            self.assertEqual(page.locator("#noiseFilterToggleBtn").inner_text(), "Noise过滤: 关")
            browser.close()

    def test_point_count_changes_do_not_reflow_sidebar(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            card = page.locator("#pointsDisplay").locator("xpath=..")
            follower = page.locator("#targetsDisplay").locator("xpath=..")
            heights = []
            follower_tops = []
            for value in (
                "10/10 (静:10/10/动:0+候:0)",
                "1000/1000 (静:1000/1000/动:1000+候:1000)",
                "0/0 (实测角度·临时)",
            ):
                page.locator("#pointsDisplay").evaluate("(el, text) => el.textContent = text", value)
                heights.append(card.bounding_box()["height"])
                follower_tops.append(follower.bounding_box()["y"])
            self.assertEqual(len(set(heights)), 1)
            self.assertEqual(len(set(follower_tops)), 1)
            browser.close()

    def test_radar_frames_do_not_overwrite_motor_status_text(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            status = {
                "type": "gimbal_scan_status",
                "enabled": True,
                "status": "running",
                "port_configured": True,
                "direction": "cw",
                "rpm": 30,
                "motor_speed_rpm": 29.9,
                "encoder_direction_detected": -1,
                "encoder_direction_verified": True,
                "live_registration": {
                    "state": "provisional",
                    "registered_frames": 70,
                    "pending_frames": 0,
                },
            }
            page.evaluate("status => handleConfigResponse(status)", status)
            display = page.locator("#gimbalScanStatusDisplay")
            stable_text = display.inner_text()
            self.assertIn("反馈 29.9 rpm", stable_text)

            frame = {
                "frame_num": 71,
                "points": [],
                "range_profile": [],
                "gimbal_scan": {
                    "enabled": True,
                    "status": "running",
                    "direction": "cw",
                    "rpm": 30,
                    "points": [],
                    "live_registration": {
                        "state": "provisional",
                        "registered_frames": 70,
                        "pending_frames": 0,
                    },
                },
            }
            page.evaluate("frame => renderRadarFrame(frame)", frame)
            self.assertEqual(display.inner_text(), stable_text)
            browser.close()

    def test_point_count_keeps_full_telemetry_in_hover_title(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            value = "1000/1000 (静:1000/1000/动:1000+候:1000)"
            page.evaluate("value => setPointsDisplay(value)", value)
            self.assertEqual(page.locator("#pointsDisplay").get_attribute("title"), value)
            browser.close()

    def test_ppi_grid_and_mapping_remain_full_circle_when_motor_is_stopped(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2048, "height": 1100})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const arcs = [];
                const originalArc = ctx.arc.bind(ctx);
                ctx.arc = (...args) => { arcs.push(args); return originalArc(...args); };
                const origin = mapToCanvas(0, 0);
                const behind = mapToCanvas(0, -1);
                document.getElementById('motorRpmInput').value = '30';
                updateMotorFrameAngleDisplay();
                renderRadarFrame({frame_num: 1, points: [{x: 0, y: 1, z: 0, v: 0}],
                    gimbal_scan: {enabled: false, mode: 'qd4310', status: 'idle'}});
                ctx.arc = originalArc;
                return {
                    origin, behind,
                    width: canvas.width, height: canvas.height,
                    fullRings: arcs.filter(a => a[2] > 100 && Math.abs(a[4] - a[3] - 2 * Math.PI) < 0.001).length,
                    frameAngle: document.getElementById('motorFrameAngleDisplay').value,
                    aspect: canvas.getBoundingClientRect().width / canvas.getBoundingClientRect().height
                };
            }""")
            browser.close()
        self.assertAlmostEqual(result["origin"]["px"], result["width"] / 2)
        self.assertAlmostEqual(result["origin"]["py"], result["height"] / 2)
        self.assertGreater(result["behind"]["py"], result["origin"]["py"])
        self.assertGreaterEqual(result["fullRings"], 4)
        self.assertEqual(result["frameAngle"], "18.00")
        self.assertAlmostEqual(result["aspect"], 1, delta=0.02)

    def test_full_circle_grid_does_not_invent_rear_returns_without_registration(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            shown = page.evaluate("""() => {
                let shown = [];
                drawPoints = (_ctx, points) => { shown = points.map(pt => pt.y); };
                renderRadarFrame({frame_num: 2, points: [
                    {x: 0, y: 1, z: 0, v: 0}, {x: 0, y: -1, z: 0, v: 0}],
                    gimbal_scan: {enabled: false, mode: 'qd4310', status: 'idle'}});
                return shown;
            }""")
            browser.close()
        self.assertEqual(shown, [1])

    def test_qd4310_rpm_input_allows_120_up_to_130(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const input = document.getElementById('motorRpmInput');
                const alerts = [];
                window.alert = message => alerts.push(message);
                input.value = '120';
                const at120 = validMotorRpm();
                input.value = '130';
                const at130 = validMotorRpm();
                input.value = '130.1';
                const above130 = validMotorRpm();
                return {at120, at130, above130, max: input.max, alerts};
            }""")
            browser.close()
        self.assertEqual(result["at120"], 120)
        self.assertEqual(result["at130"], 130)
        self.assertIsNone(result["above130"])
        self.assertEqual(result["max"], "130")
        self.assertIn("130 rpm", result["alerts"][-1])

    def test_qd4310_panel_sends_validated_start_update_and_stop(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            commands = page.evaluate("""() => {
                const commands = [];
                ws = {readyState: WebSocket.OPEN, send: raw => commands.push(JSON.parse(raw))};
                document.getElementById('motorRpmInput').value = '10';
                document.getElementById('gimbalScanToggleBtn').click();
                gimbalScanEnabled = true;
                updateGimbalScanButton();
                document.getElementById('motorRpmInput').value = '5.5';
                document.getElementById('motorApplyRpmBtn').click();
                document.getElementById('gimbalScanToggleBtn').click();
                renderRadarFrame({frame_num: 12, points: [{x: 1, y: 2, z: 0, v: 0}],
                    gimbal_scan: {enabled: true, mode: 'qd4310', rpm: 5.5,
                        status: 'running', port_configured: true}});
                handleConfigResponse({type: 'gimbal_scan_status', enabled: true,
                    mode: 'qd4310', rpm: 5.5, status: 'running', port_configured: true});
                return commands;
            }""")
            self.assertNotIn("360合成", page.locator("#pointsDisplay").inner_text())
            self.assertIn("5.5 rpm", page.locator("#gimbalScanStatusDisplay").inner_text())
            browser.close()
        self.assertEqual([item["action"] for item in commands], ["start", "set_speed", "stop"])
        self.assertEqual(commands[0]["params"]["mode"], "qd4310")
        self.assertEqual(commands[0]["params"]["rpm"], 10)
        self.assertEqual(commands[1]["params"]["rpm"], 5.5)

    def test_qd4310_direction_and_encoder_angle_are_not_inferred_from_rpm(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const commands = [];
                ws = {readyState: WebSocket.OPEN, send: raw => commands.push(JSON.parse(raw))};
                document.getElementById('motorDirectionSelect').value = 'cw';
                document.getElementById('gimbalScanToggleBtn').click();
                handleConfigResponse({type: 'gimbal_scan_status', enabled: true,
                    status: 'running', mode: 'qd4310', direction: 'cw', rpm: 2,
                    port_configured: true, motor_angle_deg: 49.446,
                    motor_angle_age_ms: 120, motor_speed_rpm: -1.9});
                const fresh = document.getElementById('motorAngleDisplay').textContent;
                const locked = document.getElementById('motorDirectionSelect').disabled;
                handleConfigResponse({type: 'gimbal_scan_status', enabled: true,
                    status: 'running', mode: 'qd4310', direction: 'cw', rpm: 2,
                    port_configured: true, motor_angle_deg: null});
                const stale = document.getElementById('motorAngleDisplay').textContent;
                return {commands, fresh, stale, locked};
            }""")
            browser.close()
        self.assertEqual(result["commands"][0]["params"]["direction"], "cw")
        self.assertTrue(result["locked"])
        self.assertIn("49.45", result["fresh"])
        self.assertNotIn("49.45", result["stale"])

    def test_motor_direction_updates_encoder_trend_default(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            self.assertEqual(page.locator("#motorDirectionSelect").input_value(), "ccw")
            self.assertEqual(page.locator("#registrationDirectionInput").input_value(), "1")
            page.locator("#motorDirectionSelect").select_option("cw")
            self.assertEqual(page.locator("#registrationDirectionInput").input_value(), "-1")
            page.locator("#motorDirectionSelect").select_option("ccw")
            self.assertEqual(page.locator("#registrationDirectionInput").input_value(), "1")
            browser.close()

    def test_qd4310_live_registration_requires_opt_in_and_can_show_rear_point(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const commands = [];
                ws = {readyState: WebSocket.OPEN, send: raw => commands.push(JSON.parse(raw))};
                const enabled = document.getElementById('liveRegistrationEnabled');
                const defaultChecked = enabled.checked;
                enabled.checked = true;
                document.getElementById('registrationDirectionInput').value = '-1';
                document.getElementById('registrationZeroInput').value = '12.5';
                document.getElementById('registrationRollInput').value = '90';
                document.getElementById('gimbalScanToggleBtn').click();
                let drawn = [];
                let scanDrawCalls = 0;
                drawGimbalScanPoints = points => { drawn = points; scanDrawCalls++; };
                renderRadarFrame({frame_num: 20, points: [{x: 0, y: 1, z: 0, v: 0}],
                    gimbal_scan: {enabled: true, mode: 'qd4310', rpm: 1,
                        status: 'running', port_configured: true,
                        live_registration: {state: 'provisional', registered_frames: 2},
                        points: [{x: 0, y: -1, z: 0, v: 0}]}});
                const provisionalLabel = document.getElementById('pointsDisplay').innerText;
                const provisionalStatus = document.getElementById('liveRegistrationStatusDisplay').innerText;
                const provisionalDrawCalls = scanDrawCalls;
                renderRadarFrame({frame_num: 21, points: [{x: 0, y: 1, z: 0, v: 0}],
                    gimbal_scan: {enabled: true, mode: 'qd4310', rpm: 1,
                        status: 'running', port_configured: true,
                        live_registration: {state: 'stale', registered_frames: 2},
                        points: [{x: 0, y: -1, z: 0, v: 0}]}});
                return {defaultChecked, command: commands[0], drawn, provisionalDrawCalls,
                    provisionalLabel, provisionalStatus,
                    staleDrawCalls: scanDrawCalls,
                    label: document.getElementById('pointsDisplay').innerText,
                    liveStatus: document.getElementById('liveRegistrationStatusDisplay').innerText};
            }""")
            browser.close()
        self.assertFalse(result["defaultChecked"])
        self.assertTrue(result["command"]["params"]["live_registration_enabled"])
        self.assertEqual(result["command"]["params"]["registration_direction"], -1)
        self.assertEqual(result["command"]["params"]["registration_zero_deg"], 12.5)
        self.assertEqual(result["command"]["params"]["registration_mounting_roll_deg"], 90)
        self.assertEqual(result["drawn"][0]["y"], -1)
        self.assertIn("实测角度", result["provisionalLabel"])
        self.assertIn("2 帧", result["provisionalStatus"])
        self.assertEqual(result["provisionalDrawCalls"], 1)
        self.assertEqual(result["staleDrawCalls"], 1)
        self.assertNotIn("实测角度", result["label"])
        self.assertIn("等待有效编码器角度", result["liveStatus"])

    def test_calibration_freeze_keeps_raw_point_and_native_camera_pixel(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.route(
                "http://synthetic-camera/**",
                lambda route: route.fulfill(
                    status=200,
                    body=camera_png(1280, 720),
                    headers={
                        "content-type": "image/png",
                        "access-control-allow-origin": "*",
                        "access-control-expose-headers": "X-Camera-Frame-Id",
                        "X-Camera-Frame-Id": "7",
                    },
                ),
            )
            page.goto(page_url, wait_until="networkidle")
            self.assertFalse(page.evaluate("freezeCalibrationSnapshot()"))
            page.evaluate(
                """renderRadarFrame({
                    frame_num: 88,
                    points: [{x: 1, y: 2, z: 0.3, v: 0}],
                    camera_sync: {status: 'matched', frame_id: 7,
                        frame_url: 'http://synthetic-camera/camera/frame/7.jpg', time_offset_ms: 12}
                })"""
            )
            page.wait_for_function("displayedCameraFrameId === 7")
            self.assertTrue(page.evaluate("freezeCalibrationSnapshot()"))
            page.evaluate("liveSpatialAnalysis.rawPoints[0].z = 99")
            picked = page.evaluate(
                """() => {
                    const {px, py} = calibrationSnapshot.pointPixels[0];
                    radarRangeX = 100;
                    radarRangeY = 100;
                    const rect = canvas.getBoundingClientRect();
                    handleLiveCanvasClick({clientX: rect.left + px * rect.width / canvas.width,
                        clientY: rect.top + py * rect.height / canvas.height});
                    const imageRect = cameraCanvas.getBoundingClientRect();
                    selectCalibrationImagePixel({clientX: imageRect.left + imageRect.width / 2,
                        clientY: imageRect.top + imageRect.height / 2});
                    return {radar: calibrationRadarSelection, image: calibrationImageSelection};
                }"""
            )
            self.assertEqual(picked["radar"]["rawIndex"], 0)
            self.assertEqual(picked["image"], {"u": 640, "v": 360})
            page.evaluate("saveCalibrationSample()")
            page.evaluate("window.frozenPpiDataUrl = canvas.toDataURL()")
            page.evaluate("renderRadarFrame({frame_num: 89, points: [], camera_sync: null})")
            self.assertTrue(page.evaluate("canvas.toDataURL() === window.frozenPpiDataUrl"))
            self.assertEqual(page.evaluate("calibrationSnapshot.radarFrameNum"), 88)
            saved = page.evaluate("calibrationSession.samples[0]")
            self.assertEqual(saved["radar"]["frame_num"], 88)
            self.assertEqual(saved["radar"]["point_index"], 0)
            self.assertEqual([saved["radar"][axis] for axis in ("x", "y", "z")], [1, 2, 0.3])
            self.assertEqual(saved["camera"], {"frame_id": 7, "u": 640, "v": 360})
            self.assertEqual(saved["set"], "fit")
            self.assertEqual(saved["sync_offset_ms"], 12)
            self.assertEqual(page.evaluate("displayedCameraFrameId"), 7)
            browser.close()

    def test_calibration_ppi_hides_ghost_canvas_and_lists_only_frozen_raw_points(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const ppiPixels = document.createElement('canvas');
                ppiPixels.width = canvas.width; ppiPixels.height = canvas.height;
                const old = ppiPixels.getContext('2d');
                old.fillStyle = '#ff0000'; old.fillRect(44, 44, 12, 12);
                calibrationSnapshot = {
                    radarFrameNum: 88, cameraFrameId: 7, cameraImageSize: [1280, 720],
                    syncOffsetMs: 12, timestamp: '2026-09-23T00:00:00Z', ppiPixels,
                    rawPoints: [
                        {x: -1.18, y: 0.72, z: -0.20, v: 0.12, snr: 12.0, noise: 5.0},
                        {x: 0.25, y: 2.80, z: 0.10, v: 0.00, snr: 25.0, noise: 7.0}
                    ]
                };
                calibrationSnapshot.pointPixels = calibrationSnapshot.rawPoints.map(
                    point => mapToCanvas(point.x, point.y));
                drawCalibrationPpi();
                renderCalibrationPointCandidates();
                const buttons = [...document.querySelectorAll('#calibrationPointCandidates button')];
                const rect = canvas.getBoundingClientRect();
                handleLiveCanvasClick({clientX: rect.left + 12 * rect.width / canvas.width,
                    clientY: rect.top + 12 * rect.height / canvas.height});
                const noHitStatus = document.getElementById('calibrationStatus').textContent;
                buttons[0].click();
                return {
                    ghostPixel: [...ctx.getImageData(50, 50, 1, 1).data],
                    labels: buttons.map(button => button.textContent),
                    selected: calibrationRadarSelection.rawIndex,
                    noHitStatus,
                    assistance: document.getElementById('calibrationCandidateAssist').textContent
                };
            }""")
            self.assertNotEqual(result["ghostPixel"][:3], [255, 0, 0])
            self.assertEqual(len(result["labels"]), 2)
            self.assertIn("#1", result["labels"][0])
            self.assertIn("SNR 25.0", result["labels"][0])
            self.assertIn("高SNR候选", result["labels"][0])
            self.assertIn("#0", result["labels"][1])
            self.assertEqual(result["selected"], 1)
            self.assertIn("余辉和历史轨迹不可选", result["noHitStatus"])
            self.assertIn("按 SNR 从高到低", result["assistance"])
            browser.close()

    def test_ppi_uses_loaded_aoa_limits_and_keeps_high_azimuth_points_visible(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                document.getElementById('f_fovRange').value = '3.5';
                document.getElementById('f_azimMin').value = '-90';
                document.getElementById('f_azimMax').value = '90';
                updateRadarDisplayRange();
                const origin = mapToCanvas(0, 0);
                const oneMetreRight = mapToCanvas(1, 0);
                const oneMetreForward = mapToCanvas(0, 1);
                return {
                    screenshotPoint: mapToCanvas(2.52, 1.60),
                    eightyFiveDegreePoint: mapToCanvas(3.45, 0.30),
                    width: canvas.width,
                    height: canvas.height,
                    xPixelsPerMetre: oneMetreRight.px - origin.px,
                    yPixelsPerMetre: origin.py - oneMetreForward.py,
                };
            }""")
            for point_name in ("screenshotPoint", "eightyFiveDegreePoint"):
                self.assertGreaterEqual(result[point_name]["px"], 14)
                self.assertLessEqual(result[point_name]["px"], result["width"] - 14)
                self.assertGreaterEqual(result[point_name]["py"], 14)
                self.assertLessEqual(result[point_name]["py"], result["height"] - 14)
            self.assertAlmostEqual(
                result["xPixelsPerMetre"], result["yPixelsPerMetre"], places=6
            )
            browser.close()

    def test_ppi_refreshes_azimuth_limits_from_loaded_aoa_config(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                parseCfgToForm('cfarFovCfg -1 0 0 8.0\\naoaFovCfg -1 -70 80 -30 30');
                return {
                    azimMinInput: document.getElementById('f_azimMin').value,
                    azimMaxInput: document.getElementById('f_azimMax').value,
                    azimMin: radarAzimuthMinDeg,
                    azimMax: radarAzimuthMaxDeg,
                };
            }""")
            self.assertEqual(result, {
                "azimMinInput": "-70", "azimMaxInput": "80",
                "azimMin": -70, "azimMax": 80,
            })
            browser.close()

    def test_calibration_candidate_assistance_is_explicit_when_side_info_is_missing(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            assistance = page.evaluate("""() => {
                calibrationSnapshot = {
                    rawPoints: [{x: 0, y: 2, z: 0, v: 0}],
                    pointPixels: [mapToCanvas(0, 2)]
                };
                renderCalibrationPointCandidates();
                return document.getElementById('calibrationCandidateAssist').textContent;
            }""")
            self.assertIn("未提供 SNR/Noise Side Info", assistance)
            self.assertIn("不能自动确认", assistance)
            browser.close()

    def test_calibration_uses_native_640_by_480_pixels_through_scaled_display_and_export(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900}, accept_downloads=True)
            page.route("http://synthetic-camera/**", lambda route: route.fulfill(
                status=200, body=camera_png(640, 480), headers={
                    "content-type": "image/png", "access-control-allow-origin": "*",
                    "access-control-expose-headers": "X-Camera-Frame-Id", "X-Camera-Frame-Id": "7"}))
            page.goto(page_url, wait_until="networkidle")
            page.evaluate("""renderRadarFrame({frame_num: 88, points: [{x: 1, y: 2, z: 0.3}],
                camera_sync: {status: 'matched', frame_id: 7,
                    frame_url: 'http://synthetic-camera/camera/frame/7.jpg', time_offset_ms: 12}})""")
            page.wait_for_function("displayedCameraFrameId === 7")
            display = page.evaluate("""() => {
                document.getElementById('cameraOverlayEnabled').checked = true;
                drawCameraProjection({status: 'valid', points: [{u: 320, v: 240}]});
                const rect = cameraCanvas.getBoundingClientRect();
                return {image: [cameraCanvas.width, cameraCanvas.height],
                    overlay: [cameraOverlayCanvas.width, cameraOverlayCanvas.height],
                    displayRatio: rect.width / rect.height,
                    projectedAlpha: cameraOverlayCtx.getImageData(320, 240, 1, 1).data[3]};
            }""")
            self.assertEqual(display["image"], [640, 480])
            self.assertEqual(display["overlay"], [640, 480])
            self.assertAlmostEqual(display["displayRatio"], 4 / 3, places=2)
            self.assertGreater(display["projectedAlpha"], 0)
            self.assertTrue(page.evaluate("freezeCalibrationSnapshot()"))
            picked = page.evaluate("""() => {
                selectCalibrationRadarPoint(calibrationSnapshot.rawPoints[0], 0);
                const rect = cameraCanvas.getBoundingClientRect();
                selectCalibrationImagePixel({clientX: rect.left + rect.width / 4,
                    clientY: rect.top + rect.height / 2});
                return {snapshotSize: calibrationSnapshot.cameraImageSize,
                    pixel: calibrationImageSelection};
            }""")
            self.assertEqual(picked, {"snapshotSize": [640, 480], "pixel": {"u": 160, "v": 240}})
            self.assertTrue(page.evaluate("saveCalibrationSample()"))
            with page.expect_download() as download_info:
                self.assertTrue(page.evaluate("downloadCalibrationSession()"))
            exported = json.loads(Path(download_info.value.path()).read_text(encoding="utf-8"))
            self.assertEqual(exported["camera_image_size"], [640, 480])
            self.assertEqual(exported["samples"][0]["camera"], {"frame_id": 7, "u": 160, "v": 240})
            browser.close()

    def test_calibration_raw_point_details_are_visible_and_exported_without_invented_values(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(accept_downloads=True)
            page.goto(page_url, wait_until="networkidle")
            details = page.evaluate("""() => {
                const ppiPixels = document.createElement('canvas');
                ppiPixels.width = canvas.width; ppiPixels.height = canvas.height;
                calibrationSnapshot = {radarFrameNum: 88, cameraFrameId: 7,
                    cameraImageSize: [1280, 720], syncOffsetMs: 12,
                    timestamp: '2026-09-23T00:00:00Z', ppiPixels,
                    rawPoints: [{x: 3, y: 4, z: 12, v: 0, snr: 18.5, noise: 7.2}],
                    pointPixels: [{px: 400, py: 300}]};
                selectCalibrationRadarPoint(calibrationSnapshot.rawPoints[0], 0);
                return document.getElementById('calibrationStatus').textContent;
            }""")
            for marker in ("x=3", "y=4", "z=12", "13", "v=0", "SNR 18.5", "Noise 7.2"):
                self.assertIn(marker, details)
            self.assertIn("x=3", page.locator("#calibrationPointSelection").inner_text())
            page.evaluate("""() => {
                const rect = cameraCanvas.getBoundingClientRect();
                selectCalibrationImagePixel({clientX: rect.left + rect.width / 2,
                    clientY: rect.top + rect.height / 2});
            }""")
            self.assertIn("x=3", page.locator("#calibrationStatus").inner_text())
            self.assertTrue(page.evaluate("saveCalibrationSample()"))
            table = page.locator("#calibrationSampleList").inner_text()
            for marker in ("3", "4", "12", "13", "18.5", "7.2"):
                self.assertIn(marker, table)
            page.evaluate("""() => {
                calibrationSnapshot.radarFrameNum = 89;
                calibrationSnapshot.cameraFrameId = 8;
                calibrationSnapshot.rawPoints = [{x: 1, y: 2, z: 0}];
                selectCalibrationRadarPoint(calibrationSnapshot.rawPoints[0], 0);
                calibrationImageSelection = {u: 641, v: 360};
            }""")
            self.assertTrue(page.evaluate("saveCalibrationSample()"))
            page.evaluate("""() => {
                const template = calibrationSession.samples[0];
                for (let index = 2; index < 7; index++) {
                    calibrationSession.samples.push({
                        ...template, id: `audit-fixture-${index}`,
                        set: index === 6 ? 'validation' : 'fit',
                        radar: {...template.radar, frame_num: 88 + index,
                            point_index: index, raw_index: index, x: 10 + index},
                        camera: {...template.camera, frame_id: 7 + index, u: 100 + index}
                    });
                }
            }""")
            with page.expect_download() as download_info:
                self.assertTrue(page.evaluate("downloadCalibrationSession()"))
            exported = json.loads(Path(download_info.value.path()).read_text(encoding="utf-8"))
            self.assertEqual(len(load_session(Path(download_info.value.path())).samples), 7)
            radar = exported["samples"][0]["radar"]
            self.assertEqual(radar, {"frame_num": 88, "point_index": 0, "raw_index": 0,
                                     "x": 3, "y": 4, "z": 12, "range_m": 13,
                                     "velocity_mps": 0, "snr": 18.5, "noise": 7.2})
            missing = exported["samples"][1]["radar"]
            self.assertEqual(missing["frame_num"], 89)
            self.assertEqual(missing["point_index"], 0)
            self.assertIsNone(missing["velocity_mps"])
            self.assertIsNone(missing["snr"])
            self.assertIsNone(missing["noise"])
            browser.close()

    def test_calibration_session_rejects_mixed_native_image_sizes(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            result = page.evaluate("""() => {
                const ppiPixels = document.createElement('canvas');
                ppiPixels.width = canvas.width; ppiPixels.height = canvas.height;
                calibrationSnapshot = {radarFrameNum: 88, cameraFrameId: 7,
                    cameraImageSize: [640, 480], syncOffsetMs: 12,
                    timestamp: '2026-09-23T00:00:00Z', ppiPixels,
                    rawPoints: [{x: 1, y: 2, z: 3}], pointPixels: [{px: 400, py: 300}]};
                selectCalibrationRadarPoint(calibrationSnapshot.rawPoints[0], 0);
                calibrationImageSelection = {u: 160, v: 240};
                const first = saveCalibrationSample();
                calibrationSnapshot.radarFrameNum = 89;
                calibrationSnapshot.cameraFrameId = 8;
                calibrationSnapshot.cameraImageSize = [1280, 720];
                selectCalibrationRadarPoint(calibrationSnapshot.rawPoints[0], 0);
                calibrationImageSelection = {u: 320, v: 360};
                const second = saveCalibrationSample();
                return {first, second, size: calibrationSession.camera_image_size,
                    count: calibrationSession.samples.length,
                    status: document.getElementById('calibrationStatus').textContent};
            }""")
            self.assertTrue(result["first"])
            self.assertFalse(result["second"])
            self.assertEqual(result["size"], [640, 480])
            self.assertEqual(result["count"], 1)
            self.assertIn("分辨率", result["status"])
            browser.close()

    def test_calibration_rejects_duplicate_pairs_and_requires_new_validation_frame(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            outcomes = page.evaluate(
                """() => {
                    const ppiPixels = document.createElement('canvas');
                    ppiPixels.width = canvas.width;
                    ppiPixels.height = canvas.height;
                    calibrationSnapshot = {
                        radarFrameNum: 88, cameraFrameId: 7, syncOffsetMs: 12,
                        timestamp: '2026-09-23T00:00:00Z', ppiPixels,
                        rawPoints: [{x: 1, y: 2, z: 0.3}], pointPixels: [{px: 400, py: 300}]
                    };
                    const sampleSet = document.getElementById('calibrationSampleSet');
                    const choose = (u = 640, v = 360) => {
                        calibrationRadarSelection = {point: {x: 1, y: 2, z: 0.3}, rawIndex: 0};
                        calibrationImageSelection = {u, v};
                    };
                    choose();
                    const first = saveCalibrationSample();
                    choose();
                    const sameSet = saveCalibrationSample();
                    const sameSetStatus = document.getElementById('calibrationStatus').textContent;
                    sampleSet.value = 'validation';
                    choose();
                    const crossSet = saveCalibrationSample();
                    const crossSetStatus = document.getElementById('calibrationStatus').textContent;
                    choose(641, 360);
                    const sameFrame = saveCalibrationSample();
                    calibrationSnapshot.radarFrameNum = 89;
                    calibrationSnapshot.cameraFrameId = 8;
                    choose();
                    const newFrame = saveCalibrationSample();
                    return {first, sameSet, sameSetStatus, crossSet, crossSetStatus,
                        sameFrame, newFrame, sets: calibrationSession.samples.map(sample => sample.set)};
                }"""
            )
            self.assertTrue(outcomes["first"])
            self.assertFalse(outcomes["sameSet"])
            self.assertIn("重复", outcomes["sameSetStatus"])
            self.assertFalse(outcomes["crossSet"])
            self.assertIn("独立", outcomes["crossSetStatus"])
            self.assertFalse(outcomes["sameFrame"])
            self.assertTrue(outcomes["newFrame"])
            self.assertEqual(outcomes["sets"], ["fit", "validation"])
            browser.close()

    def test_calibration_export_mount_requires_complete_finite_measurement(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page()
            page.goto(page_url, wait_until="networkidle")
            page.evaluate("""() => {
                calibrationSession.samples = Array.from({length: 7}, (_, index) => ({
                    id: `fixture-${index}`, set: index === 6 ? 'validation' : 'fit',
                    radar: {frame_num: 88 + index, point_index: index, x: 1 + index, y: 2, z: 0.3},
                    camera: {frame_id: 7 + index, u: 640 + index, v: 360},
                    sync_offset_ms: 12, timestamp: '2026-09-23T00:00:00Z', note: ''
                }));
            }""")
            page.evaluate("""() => {
                document.getElementById('mountDxInput').value = '0';
                document.getElementById('mountDyInput').value = '0';
                document.getElementById('mountDzInput').value = '-0.1';
            }""")
            self.assertFalse(page.evaluate("downloadCalibrationSession()"))
            page.evaluate("document.getElementById('mountUncertaintyInput').value = '0.02'")
            mount = page.evaluate("buildCalibrationMountMeasurement()")
            self.assertEqual(mount, {"dx_m": 0, "dy_m": 0, "dz_m": -0.1,
                                     "uncertainty_m": 0.02,
                                     "reference": "radar phase centre to camera optical centre"})
            with page.expect_download() as download_info:
                self.assertTrue(page.evaluate("downloadCalibrationSession()"))
            exported = json.loads(Path(download_info.value.path()).read_text(encoding="utf-8"))
            self.assertEqual(exported["mount_measurement"], mount)
            session = load_session(Path(download_info.value.path()))
            self.assertEqual(session.mount_measurement, (0.0, 0.0, -0.1))
            self.assertEqual(session.mount_uncertainty_m, 0.02)
            page.evaluate("document.getElementById('mountUncertaintyInput').value = '-0.02'")
            self.assertFalse(page.evaluate("downloadCalibrationSession()"))
            browser.close()

    def test_recording_start_sends_idle_registration_calibration_without_starting_motor(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.add_init_script("""
                window.sentCommands = [];
                class FakeWebSocket {
                    static OPEN = 1;
                    constructor() { this.readyState = 1; setTimeout(() => this.onopen && this.onopen(), 0); }
                    send(payload) { window.sentCommands.push(JSON.parse(payload)); }
                    close() { this.readyState = 3; if (this.onclose) this.onclose(); }
                }
                window.WebSocket = FakeWebSocket;
            """)
            page.goto(page_url, wait_until="networkidle")
            page.locator("#wsInput").fill("127.0.0.1:8765")
            page.locator("#connectBtn").click()
            page.wait_for_function("window.sentCommands.some(item => item.type === 'list_configs')")
            page.locator("#liveRegistrationEnabled").check()
            page.locator("#registrationRollInput").fill("90")
            page.locator("#recordPointcloudBtn").click()
            commands = page.evaluate("window.sentCommands")
            recording = [item for item in commands if item.get("type") == "pointcloud_record" and item.get("action") == "start"]
            self.assertEqual(len(recording), 1)
            self.assertEqual(recording[0]["registration"]["mounting_roll_deg"], 90)
            self.assertTrue(recording[0]["registration"]["enabled"])
            self.assertFalse(any(item.get("type") == "gimbal_scan" and item.get("action") == "start" for item in commands))
            browser.close()

    def test_offline_replay_controls_render_without_javascript_errors(self):
        page_errors = []
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.on("pageerror", lambda error: page_errors.append(str(error)))
            page.goto(page_url, wait_until="networkidle")
            for selector in (
                "#replayPanel",
                "#refreshReplayBtn",
                "#replayCaptureSelect",
                "#replayPreviousBtn",
                "#replayPlayBtn",
                "#replayNextBtn",
                "#replayStatusDisplay",
            ):
                self.assertEqual(page.locator(selector).count(), 1, selector)
            self.assertTrue(page.locator("#replayPlayBtn").is_disabled())
            browser.close()
        self.assertEqual(page_errors, [])

    def test_matched_radar_frame_fetches_and_displays_mock_camera_image(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.route(
                "http://synthetic-camera/**",
                lambda route: route.fulfill(
                    status=200,
                    body=CAMERA_IMAGE,
                    headers={
                        "content-type": "image/gif",
                        "access-control-allow-origin": "*",
                        "access-control-expose-headers": "X-Camera-Frame-Id, X-Capture-Monotonic-Ns, X-Capture-Wall-Time-Ns",
                        "X-Camera-Frame-Id": "7",
                        "X-Capture-Monotonic-Ns": "2000000000",
                        "X-Capture-Wall-Time-Ns": "1700000000000000000",
                    },
                ),
            )
            page.goto(page_url, wait_until="networkidle")
            self.assertEqual(page.locator("#cameraDisplayMode").input_value(), "matched")
            page.evaluate(
                """renderRadarFrame({
                    frame_num: 1,
                    points: [],
                    camera_sync: {
                        status: 'matched',
                        frame_id: 7,
                        frame_url: 'http://synthetic-camera/camera/frame/7.jpg',
                        time_offset_ms: 20
                    },
                    camera_projection: { status: 'unavailable', reason: 'calibration_not_loaded' }
                })"""
            )
            page.wait_for_function(
                "document.getElementById('cameraFrameId').innerText.startsWith('#7')",
                timeout=5_000,
            )
            self.assertEqual(page.locator("#cameraStatus").inner_text(), "matched")
            self.assertEqual(page.locator("#cameraSyncOffset").inner_text(), "20.0 ms")
            self.assertTrue(page.locator("#cameraFrameId").inner_text().startswith("#7 / "))
            browser.close()

    def test_mismatched_camera_http_frame_is_not_presented_as_the_radar_match(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.route(
                "http://synthetic-camera/**",
                lambda route: route.fulfill(
                    status=200,
                    body=CAMERA_IMAGE,
                    headers={
                        "content-type": "image/gif",
                        "access-control-allow-origin": "*",
                        "access-control-expose-headers": "X-Camera-Frame-Id, X-Capture-Monotonic-Ns, X-Capture-Wall-Time-Ns",
                        "X-Camera-Frame-Id": "6",
                        "X-Capture-Monotonic-Ns": "2000000000",
                        "X-Capture-Wall-Time-Ns": "1700000000000000000",
                    },
                ),
            )
            page.goto(page_url, wait_until="networkidle")
            page.evaluate(
                """renderRadarFrame({
                    frame_num: 3,
                    points: [],
                    camera_sync: {
                        status: 'matched',
                        frame_id: 7,
                        frame_url: 'http://synthetic-camera/camera/frame/7.jpg',
                        time_offset_ms: 10
                    }
                })"""
            )
            page.wait_for_function("document.getElementById('cameraStatus').innerText === 'error'")
            self.assertFalse(page.locator("#cameraFrameId").inner_text().startswith("#6 / "))
            self.assertFalse(page.evaluate("freezeCalibrationSnapshot()"))
            browser.close()

    def test_camera_http_error_is_shown_without_a_page_exception(self):
        page_errors = []
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.on("pageerror", lambda error: page_errors.append(str(error)))
            page.route("http://synthetic-camera/**", lambda route: route.fulfill(status=404, body="missing"))
            page.goto(page_url, wait_until="networkidle")
            page.evaluate(
                """renderRadarFrame({
                    frame_num: 2,
                    points: [],
                    camera_sync: {
                        status: 'matched',
                        frame_id: 8,
                        frame_url: 'http://synthetic-camera/camera/frame/8.jpg',
                        time_offset_ms: -20
                    }
                })"""
            )
            page.wait_for_function("document.getElementById('cameraStatus').innerText === 'error'")
            self.assertEqual(page.locator("#cameraStatus").inner_text(), "error")
            browser.close()
        self.assertEqual(page_errors, [])

    def test_radar_frame_with_null_camera_sync_completes_ppi_frame_handling(self):
        """Radar-only frames must not let an absent camera halt the PPI renderer."""
        page_errors = []
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.on("pageerror", lambda error: page_errors.append(str(error)))
            page.goto(page_url, wait_until="networkidle")
            page.evaluate(
                """renderRadarFrame({
                    frame_num: 42,
                    points: [{x: 1.0, y: 2.0, z: 0.0, v: 0.0}],
                    camera_sync: null,
                    camera_projection: null
                })"""
            )
            self.assertIn("42", page.locator("#frameDisplay").inner_text())
            self.assertIn("/1", page.locator("#pointsDisplay").inner_text())
            self.assertEqual(page.locator("#cameraStatus").inner_text(), "disabled")
            browser.close()
        self.assertEqual(page_errors, [])

    def test_ppi_displays_noise_qualified_static_point_when_line_filter_is_enabled(self):
        """The PPI must remain an operator view of raw accepted radar detections."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            page.evaluate(
                """lineFilterEnabled = true; renderRadarFrame({
                    frame_num: 43,
                    points: [{x: 1.0, y: 2.0, z: 0.0, v: 0.12}],
                    camera_sync: null,
                    camera_projection: null
                })"""
            )
            self.assertTrue(page.locator("#pointsDisplay").inner_text().startswith("1/1"))
            browser.close()

    def test_raw_points_are_drawn_after_live_cluster_overlays(self):
        """Live object labels must not obscure the PPI's raw radar detections."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1440, "height": 900})
            page.goto(page_url, wait_until="networkidle")
            order = page.evaluate(
                """() => {
                    const calls = [];
                    drawPoints = () => calls.push('points');
                    drawLiveClusterOverlay = () => calls.push('clusters');
                    lineFilterEnabled = false;
                    liveObjectEnabled = true;
                    renderRadarFrame({
                        frame_num: 44,
                        points: [
                            {x: 1.00, y: 2.00, z: 0.0, v: 0.12},
                            {x: 1.05, y: 2.03, z: 0.0, v: 0.12},
                            {x: 1.10, y: 2.06, z: 0.0, v: 0.12}
                        ],
                        camera_sync: null,
                        camera_projection: null
                    });
                    return calls;
                }"""
            )
            self.assertLess(order.index("clusters"), order.index("points"))
            browser.close()

    def test_collapsed_configuration_gives_radar_and_camera_similar_large_widths(self):
        """When configuration is collapsed, the two live views should use the freed workspace."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2048, "height": 1100})
            page.goto(page_url, wait_until="networkidle")
            page.evaluate("document.getElementById('configPanel').classList.add('collapsed')")
            sizes = page.evaluate(
                """() => ({
                    radar: document.getElementById('radarCanvas').getBoundingClientRect().width,
                    radarPanel: document.querySelector('.main-visual').getBoundingClientRect().width,
                    camera: document.querySelector('.camera-panel').getBoundingClientRect().width
                })"""
            )
            self.assertGreaterEqual(sizes["radar"], 700)
            self.assertGreaterEqual(sizes["camera"], 700)
            # The square PPI is height-limited; its panel, not its canvas, shares
            # the same column width as the camera panel.
            self.assertLessEqual(abs(sizes["radarPanel"] - sizes["camera"]), 36)
            browser.close()

    def test_sidebar_can_collapse_to_expand_live_radar_and_camera_views(self):
        """The operator can reclaim the control column for the two live displays."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2048, "height": 1100})
            page.goto(page_url, wait_until="networkidle")
            page.evaluate("document.getElementById('configPanel').classList.add('collapsed')")
            before = page.locator(".main-visual").bounding_box()["width"]
            page.locator("#sidebarToggleBtn").click()
            after = page.locator(".main-visual").bounding_box()["width"]
            self.assertFalse(page.locator(".hud-sidebar").is_visible())
            self.assertGreater(after, before)
            self.assertEqual(page.locator("#sidebarToggleBtn").get_attribute("aria-label"), "展开控制栏")
            browser.close()

    def test_sidebar_toggle_is_compact_and_outside_controls_at_desktop_and_mobile_widths(self):
        """The minimize control must not cover replay buttons or other sidebar content."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            for width in (2048, 680):
                page = browser.new_page(viewport={"width": width, "height": 900})
                page.goto(page_url, wait_until="networkidle")
                toggle = page.locator("#sidebarToggleBtn")
                sidebar = page.locator(".hud-sidebar")
                toggle_box = toggle.bounding_box()
                sidebar_box = sidebar.bounding_box()
                self.assertLessEqual(toggle_box["width"], 30)
                self.assertLessEqual(toggle_box["height"], 30)
                self.assertLessEqual(toggle_box["x"] + toggle_box["width"], sidebar_box["x"])
                self.assertEqual(toggle.get_attribute("aria-label"), "收起控制栏")
                toggle.click()
                self.assertEqual(toggle.get_attribute("aria-label"), "展开控制栏")
                self.assertTrue(page.locator(".main-visual").is_visible())
                page.close()
            browser.close()

    def test_primary_views_expand_when_the_other_view_is_collapsed(self):
        """Either presentation view fills the shared display area on its own."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2048, "height": 1100})
            page.goto(page_url, wait_until="domcontentloaded")
            page.evaluate("document.getElementById('configPanel').classList.add('collapsed')")
            radar = page.locator(".main-visual")
            camera = page.locator(".camera-panel")
            radar_button = page.locator("#radarViewToggleBtn")
            camera_button = page.locator("#cameraViewToggleBtn")
            radar_width = radar.bounding_box()["width"]
            camera_width = camera.bounding_box()["width"]
            radar_canvas_width = page.locator("#radarCanvas").bounding_box()["width"]
            camera_stage_width = page.locator(".camera-stage").bounding_box()["width"]
            self.assertLess(abs(radar_width - camera_width), 40)

            camera_button.click()
            self.assertFalse(camera.is_visible())
            self.assertGreater(radar.bounding_box()["width"], radar_width * 1.7)
            canvas_box = page.locator("#radarCanvas").bounding_box()
            badge_box = page.locator("#localMapBadge").bounding_box()
            toolbar_box = page.locator(".ppi-view-toolbar").bounding_box()
            profile_box = page.locator(".profile-panel").bounding_box()
            self.assertGreater(canvas_box["width"], radar_canvas_width * 1.2)
            for box in (badge_box, toolbar_box, profile_box):
                self.assertGreater(box["x"], canvas_box["x"] + canvas_box["width"])
            self.assertLess(badge_box["y"], toolbar_box["y"])
            self.assertLess(toolbar_box["y"], profile_box["y"])
            self.assertEqual(camera_button.get_attribute("aria-label"), "展开相机视图")
            self.assertTrue(radar_button.is_disabled())
            camera_button.click()
            self.assertTrue(camera.is_visible())

            radar_button.click()
            self.assertFalse(radar.is_visible())
            self.assertGreater(camera.bounding_box()["width"], camera_width * 1.7)
            self.assertGreater(
                page.locator(".camera-stage").bounding_box()["width"],
                camera_stage_width * 1.1,
            )
            self.assertEqual(radar_button.get_attribute("aria-label"), "展开雷达视图")
            self.assertTrue(camera_button.is_disabled())
            page.locator("#sidebarToggleBtn").click()
            self.assertTrue(camera.is_visible())
            radar_button.click()
            self.assertTrue(radar.is_visible())
            browser.close()

    def test_primary_view_toggles_remain_available_on_narrow_layout(self):
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 680, "height": 900})
            page.goto(page_url, wait_until="domcontentloaded")
            page.locator("#cameraViewToggleBtn").click()
            self.assertTrue(page.locator(".main-visual").is_visible())
            self.assertFalse(page.locator(".camera-panel").is_visible())
            page.locator("#cameraViewToggleBtn").click()
            self.assertTrue(page.locator(".camera-panel").is_visible())
            browser.close()

    def test_solo_radar_uses_page_height_when_config_panel_is_hidden(self):
        """A hidden configuration panel must not reserve width and shrink the solo PPI."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2492, "height": 1263})
            page.goto(page_url, wait_until="domcontentloaded")
            self.assertFalse(page.locator("#configPanel").is_visible())
            page.locator("#cameraViewToggleBtn").click()
            canvas_box = page.locator("#radarCanvas").bounding_box()
            layout_box = page.locator(".radar-layout").bounding_box()
            self.assertGreater(canvas_box["height"], 950)
            self.assertGreater(layout_box["x"] + layout_box["width"], 2400)
            self.assertGreater(canvas_box["y"] + canvas_box["height"], 1160)
            self.assertLessEqual(canvas_box["y"] + canvas_box["height"], 1263)
            page.evaluate("document.getElementById('configPanel').style.display = 'block'")
            self.assertLessEqual(
                page.locator("#radarCanvas").bounding_box()["x"]
                + page.locator("#radarCanvas").bounding_box()["width"],
                page.locator("#configPanel").bounding_box()["x"],
            )
            browser.close()

    def test_manual_calibration_shortcut_opens_hidden_analysis_workspace(self):
        """Calibration must remain discoverable when the configuration panel is collapsed."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 1920, "height": 1080})
            page.goto(page_url)
            page.evaluate("document.getElementById('configPanel').classList.add('collapsed')")

            page.click("#calibrationShortcutBtn")

            self.assertFalse(page.locator("#configPanel").evaluate("node => node.classList.contains('collapsed')"))
            self.assertEqual(page.locator("#panelHeading").inner_text(), "点云分析工作台")
            self.assertTrue(page.locator("#analysisContent").is_visible())
            self.assertTrue(page.locator("#calibrationWorkspace").evaluate("node => node.open"))
            browser.close()

    def test_live_panels_keep_their_bottom_edges_aligned_with_the_workspace(self):
        """The Range Profile and camera panel should visually finish at the workspace bottom."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2048, "height": 1100})
            page.goto(page_url, wait_until="networkidle")
            page.evaluate("document.getElementById('configPanel').classList.add('collapsed')")
            positions = page.evaluate(
                """() => {
                    const bottom = selector => document.querySelector(selector).getBoundingClientRect().bottom;
                    return {
                        workspace: bottom('.radar-layout'),
                        profile: bottom('.profile-panel'),
                        camera: bottom('.camera-panel')
                    };
                }"""
            )
            self.assertLessEqual(abs(positions["workspace"] - positions["profile"]), 2)
            self.assertLessEqual(abs(positions["workspace"] - positions["camera"]), 2)
            browser.close()


if __name__ == "__main__":
    unittest.main()
