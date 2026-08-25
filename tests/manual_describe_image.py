"""
manual_describe_image.py

Not an automated test -- makes a real API call, so it costs money and
needs ANTHROPIC_API_KEY set. This is the actual Phase 1 sim milestone:
point it at a phone photo of a room (or a stock photo) and check whether
the description is sensible.

Run with: python -m tests.manual_describe_image path/to/photo.jpg
"""

import sys
import json
from brain.vision import describe_image


def main():
    if len(sys.argv) != 2:
        print("Usage: python -m tests.manual_describe_image path/to/photo.jpg")
        sys.exit(1)

    result = describe_image(sys.argv[1])
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
