import unittest
from pathlib import Path


try:
    from playwright.sync_api import sync_playwright
except ImportError:  # pragma: no cover - development environment may not bundle Playwright
    sync_playwright = None


class RadarAppMarkupTests(unittest.TestCase):
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
            'data-filter-tab="signal"',
            'data-filter-tab="near"',
            'data-filter-tab="line"',
            'id="filterPanelNear"',
            "switchFilterTab",
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


@unittest.skipUnless(sync_playwright is not None, "Playwright is not installed")
class RadarAppTests(unittest.TestCase):
    def test_offline_replay_controls_render_without_javascript_errors(self):
        page_errors = []
        page_url = (Path(__file__).resolve().parents[1] / "radar_app.html").as_uri()
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True)
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


if __name__ == "__main__":
    unittest.main()
