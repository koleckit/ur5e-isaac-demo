import json
import sys
from pathlib import Path

for filename in sys.argv[1:]:
    path = Path(filename)
    result = json.loads(path.read_text())
    if result.get('result') != 'PASS':
        raise RuntimeError(f'DoD did not pass: {path}: {result}')
    print(f'PASS: {path.name}')
