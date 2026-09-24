"""Contract checks for the manual corner-reflector calibration instructions."""

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNBOOK = ROOT / "docs" / "radar_camera_calibration.md"
LEDGER = ROOT / "docs" / "validation" / "radar-camera-template.md"


class RadarCameraCalibrationDocsTests(unittest.TestCase):
    def test_runbook_describes_direct_collection_and_mount_axis_proof(self):
        text = RUNBOOK.read_text(encoding="utf-8")
        for required in (
            "5 cm 角反射器", "雷达相位中心", "相机光心", "dx_m", "dy_m", "dz_m",
            "向右为正", "向前为正", "向上为正", "dz_m = -0.10",
            "抬高", "降低", "原始 z", "冻结匹配帧", "原始雷达点",
            "角反射器的相机像素中心", "内参 RMS", "≤1.5 px",
            "12–20", "6–10", "fit", "validation", "不同位置",
        ):
            with self.subTest(required=required):
                self.assertIn(required, text)
        self.assertNotIn("pairs.csv", text)
        self.assertNotIn("AprilTag", text)

    def test_runbook_gives_session_solver_command_and_independent_release_gate(self):
        text = RUNBOOK.read_text(encoding="utf-8")
        self.assertRegex(
            text,
            r"\.\\\.venv\\Scripts\\python\.exe\s+tools\\fusion\\calibrate_radar_camera\.py\s+"
            r"\$session\s+\$intrinsics\s+\$output\s+--mount-mode\s+co_rotating",
        )
        for required in (
            "calibration_candidate.report.json", "camera_center_in_radar_m",
            "translation_m", "residual_m", "median_px", "p95_px", "8 px", "20 px",
            "SHA-256", "本地", "浏览器模拟", "现场", "--camera-calibration",
            "$solverExit = $LASTEXITCODE", "Get-FileHash -Algorithm SHA256 $report",
            "Get-FileHash -Algorithm SHA256 $output", "$runId = Get-Date",
            "New-Item -ItemType Directory", "只审阅本次 `$runDir`",
            "不得读取其他运行目录中的旧文件",
        ):
            with self.subTest(required=required):
                self.assertIn(required, text)
        self.assertRegex(text, r"validation[^\n]*median_px[^\n]*≤\s*8 px")
        self.assertRegex(text, r"validation[^\n]*p95_px[^\n]*≤\s*20 px")

    def test_validation_ledger_has_fillable_calibration_evidence(self):
        text = LEDGER.read_text(encoding="utf-8")
        section = text.split("## 5. 静态标定与独立验证", 1)[1].split("## 6.", 1)[0]
        for required in (
            "dx_m / dy_m / dz_m", "测量不确定度", "相位中心", "光心",
            "抬高/降低", "原始 z", "会话路径", "会话 SHA-256",
            "内参路径", "内参 SHA-256", "fit 样本数", "validation 样本数",
            "fit RMS", "validation median_px", "validation p95_px",
            "camera_center_in_radar_m", "residual_m", ".report.json",
            "标定 JSON SHA-256", "部署提交", "部署校验和",
        ):
            with self.subTest(required=required):
                self.assertIn(required, section)
        for row in section.splitlines():
            if re.search(r"\|.*(?:SHA-256|median_px|p95_px|部署提交|部署校验和).*[|]", row):
                self.assertTrue("待填写" in row or "未运行" in row, row)
        self.assertIn("本地软件状态", text)
        self.assertIn("待现场验收", text)


if __name__ == "__main__":
    unittest.main()
