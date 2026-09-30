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
                        {x: -1.18, y: 0.72, z: -0.20, v: 0.12},
                        {x: 0.25, y: 2.80, z: 0.10, v: 0.00}
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
                buttons[1].click();
                return {
                    ghostPixel: [...ctx.getImageData(50, 50, 1, 1).data],
                    labels: buttons.map(button => button.textContent),
                    selected: calibrationRadarSelection.rawIndex,
                    noHitStatus
                };
            }""")
            self.assertNotEqual(result["ghostPixel"][:3], [255, 0, 0])
            self.assertEqual(len(result["labels"]), 2)
            self.assertIn("#0", result["labels"][0])
            self.assertIn("1.40", result["labels"][0])
            self.assertIn("#1", result["labels"][1])
            self.assertEqual(result["selected"], 1)
            self.assertIn("余辉和历史轨迹不可选", result["noHitStatus"])
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
                    camera: document.querySelector('.camera-panel').getBoundingClientRect().width
                })"""
            )
            self.assertGreaterEqual(sizes["radar"], 700)
            self.assertGreaterEqual(sizes["camera"], 700)
            self.assertLessEqual(abs(sizes["radar"] - sizes["camera"]), 36)
            browser.close()

    def test_sidebar_can_collapse_to_expand_live_radar_and_camera_views(self):
        """The operator can reclaim the control column for the two live displays."""
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = self._new_browser(playwright)
            page = browser.new_page(viewport={"width": 2048, "height": 1100})
            page.goto(page_url, wait_until="networkidle")
            page.evaluate("document.getElementById('configPanel').classList.add('collapsed')")
            before = page.locator("#radarCanvas").bounding_box()["width"]
            page.locator("#sidebarToggleBtn").click()
            after = page.locator("#radarCanvas").bounding_box()["width"]
            self.assertFalse(page.locator(".hud-sidebar").is_visible())
            self.assertGreater(after, before)
            self.assertEqual(page.locator("#sidebarToggleBtn").inner_text(), "控制栏: 展开")
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
