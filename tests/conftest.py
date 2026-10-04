# Tests must never write into the bridge's real command log (~/ebo_bridge/runtime/logs).
import os
import tempfile

os.environ.setdefault("EBO_LOG_DIR", tempfile.mkdtemp(prefix="ebolog-test-"))
