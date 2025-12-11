# Drake Installation (Ubuntu 24.04)

## Install

```bash
python3 -m venv drake-env
source drake-env/bin/activate
pip install drake ipython manipulation jupyter --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/
```

## Verify

```bash
pip show drake
```

Expected output:
```
Name: drake
Version: 1.47.0
...
```

