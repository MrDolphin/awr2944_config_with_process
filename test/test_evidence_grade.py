import csv
import json
import tempfile
import unittest
from pathlib import Path

from simulation.run_v04_31_evidence_grade import run


class EvidenceGradeTest(unittest.TestCase):
    def test_combines_pcb_cfg_and_schematic_support_but_keeps_candidate_status(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            mapping = root / "mapping.csv"
            mapping.write_text("virtual_input_index,tx_name,rx_name,cfg_row,cfg_column\n0,TX1,RX1,0,0\n", encoding="utf-8")
            regions = root / "regions.csv"
            regions.write_text("antenna,center_x_mm,center_y_mm\nTX1,1,1\nRX1,0,1\n", encoding="utf-8")
            documents = root / "documents.csv"
            documents.write_text("path,format,text_status,page,tokens,snippets,evidence_status\nschematic.pdf,pdf,text_extracted,2,TX1 RX1,50 ohm GCPW traces to antenna,pdf_text_context\n", encoding="utf-8")
            output = root / "output"
            summary = run(mapping, regions, documents, output)
            self.assertTrue(summary["schematic_gcpw_support"])
            self.assertFalse(summary["real_array_confirmed"])
            with (output / "evidence_grade.csv").open(encoding="utf-8", newline="") as handle:
                row = next(csv.DictReader(handle))
            self.assertEqual(row["channel_mapping_evidence"], "document_supported_candidate")
            self.assertEqual(row["overall_status"], "candidate_only")
            contract = json.loads((output / "calibration_input_contract.json").read_text(encoding="utf-8"))
            self.assertIn("phase_center", contract["required_for_real_aoa"])


if __name__ == "__main__":
    unittest.main()
