import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class PiNetworkModeScriptTests(unittest.TestCase):
    def test_script_exposes_status_hotspot_and_saved_wifi_modes(self):
        script = (ROOT / "tools" / "pi_network_mode.sh").read_text(encoding="utf-8")

        self.assertIn("Usage: sudo tools/pi_network_mode.sh", script)
        self.assertIn("status)", script)
        self.assertIn("hotspot)", script)
        self.assertIn("wifi)", script)
        self.assertIn('HOTSPOT_PROFILE="radar-pi-ap"', script)
        self.assertIn('WLAN_DEVICE="wlan0"', script)

    def test_script_only_operates_on_wlan0_and_preserves_dca_eth0(self):
        script = (ROOT / "tools" / "pi_network_mode.sh").read_text(encoding="utf-8")

        self.assertIn('nmcli connection up "$HOTSPOT_PROFILE" ifname "$WLAN_DEVICE"', script)
        self.assertIn('nmcli connection up "$profile" ifname "$WLAN_DEVICE"', script)
        self.assertIn('ip -4 addr show dev eth0', script)
        self.assertNotIn("nmcli connection modify dca1000", script)
        self.assertNotIn("nmcli connection down dca1000", script)
        self.assertNotIn("ip addr flush dev eth0", script)

    def test_script_requires_existing_wifi_profiles_instead_of_embedding_credentials(self):
        script = (ROOT / "tools" / "pi_network_mode.sh").read_text(encoding="utf-8")

        self.assertIn('Connection profile does not exist', script)
        self.assertIn('Profile is not a Wi-Fi connection', script)
        self.assertNotIn("password ", script.lower())
        self.assertNotIn("psk", script.lower())

    def test_help_is_available_before_network_manager_is_checked(self):
        script = (ROOT / "tools" / "pi_network_mode.sh").read_text(encoding="utf-8")

        help_case = script.index("-h|--help|help)")
        nmcli_check = script.index("require_command nmcli")
        self.assertLess(help_case, nmcli_check)


if __name__ == "__main__":
    unittest.main()
