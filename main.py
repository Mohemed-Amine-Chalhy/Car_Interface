"""Compatibility entry point for installed development environments.

The supported commands are ``car-interface`` and ``python -m car_interface``.
This wrapper remains temporarily so existing shortcuts fail with a useful
package import instead of starting the retired prototype UI.
"""

import sys

from car_interface.cli import main

if __name__ == "__main__":
    arguments = ("run",) if getattr(sys, "frozen", False) and len(sys.argv) == 1 else None
    raise SystemExit(main(arguments))
