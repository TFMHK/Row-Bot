#!/usr/bin/env bash
# sync-docs.sh — copy the web UI into /docs so GitHub Pages can serve it as a
# static, serverless site (Settings -> Pages -> Deploy from branch: main /docs).
# Re-run this whenever host/web changes, then commit + push.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$ROOT/host/web"
DEST="$ROOT/docs"
mkdir -p "$DEST"
cp "$SRC/index.html" "$DEST/index.html"
cp "$SRC/styles.css" "$DEST/styles.css"
cp "$SRC/app.js" "$DEST/app.js"
cp "$SRC/serial-bridge.js" "$DEST/serial-bridge.js"
echo "Synced host/web -> docs/"
