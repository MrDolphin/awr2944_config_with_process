import unittest
from pathlib import Path


try:
    from playwright.sync_api import sync_playwright
except ImportError:  # pragma: no cover - development environment may not bundle Playwright
    sync_playwright = None


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
