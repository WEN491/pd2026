from sentry_rl.config import load_config
from sentry_rl.utils.versioning import executor_fingerprint


def test_executor_hashes_exist():
    cfg = load_config("configs/sentry_2027.yaml")
    fp = executor_fingerprint(cfg)
    assert fp["bt_xml_sha256"]
    assert fp["decision_switch_sha256"]
