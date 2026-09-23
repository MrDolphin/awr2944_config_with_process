import base64
import unittest
from pathlib import Path


try:
    from playwright.sync_api import sync_playwright
except ImportError:  # pragma: no cover - development environment may not bundle Playwright
    sync_playwright = None


CHROME_EXECUTABLE = Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe")
CAMERA_IMAGE = base64.b64decode(
    "R0lGODlhAQABAIAAAAAAAP///ywAAAAAAQABAAACAUwAOw=="
)


class RadarAppMarkupTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
