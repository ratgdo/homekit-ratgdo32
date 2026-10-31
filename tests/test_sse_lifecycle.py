"""Compile the production lifecycle and locking code with host concurrency I/O."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SSELifecycle(unittest.TestCase):
    def test_actual_lifecycle_and_lock(self):
        source = (ROOT / "src/web.cpp").read_text()
        lifecycle = source[source.index("// SSE lifecycle helpers are local"):
                           source.index("struct SSESubscription\n{")]
        lock = source[source.index("// All subscription fields and clients"):
                      source.index("// Performance monitoring")]
        for fixture in ["sse_lifecycle_test.cpp", "sse_lock_test.cpp"]:
            with self.subTest(fixture=fixture), tempfile.TemporaryDirectory() as directory:
                code = (ROOT / "tests" / fixture).read_text()
                code = code.replace("// SOURCE_LIFECYCLE_HELPERS",
                                    "#include <atomic>\n#include <cstdint>\n" + lifecycle)
                code = code.replace("// SOURCE_SSE_LOCK", lock)
                cpp = Path(directory) / "sse.cpp"
                binary = Path(directory) / "sse"
                cpp.write_text(code)
                subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pthread",
                                "-fsanitize=address,undefined", str(cpp), "-o", str(binary)], check=True)
                subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
