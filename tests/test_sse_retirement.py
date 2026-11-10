"""Compile actual SSE retirement code with bounded, shared socket owners."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SSERetirement(unittest.TestCase):
    def test_full_registry_preserves_retired_owners(self):
        source = (ROOT / "src/web.cpp").read_text()
        sections = {
            "HTTP_ROOM": ("bool httpOwnedRoom()", "// Return false before any WebServer"),
            "SSE_HELPERS": ("// SSE lifecycle helpers are local", "// Performance monitoring"),
            "SSE_REMOVE": ("void removeSSEsubscription(SSESubscription *s)", "void SSEheartbeat(SSESubscription *s)\n{"),
        }
        fixture = (ROOT / "tests/sse_retirement_test.cpp").read_text()
        for name, (first, last) in sections.items():
            start = source.index(first)
            fixture = fixture.replace("// SOURCE_" + name,
                                      source[start:source.index(last, start)])
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "retirement.cpp"
            binary = Path(directory) / "retirement"
            cpp.write_text(fixture)
            subprocess.run(["c++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-pthread", str(cpp),
                            "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
