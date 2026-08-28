#!/usr/bin/env bash
# Vendors ESPHome's core `modbus` component and applies the transport patch.
#
# The patch is three `virtual` keywords. Everything else is upstream, byte for byte,
# so re-running this after an ESPHome update is the whole maintenance burden.
# If either substitution stops matching, the script fails loudly rather than
# producing a copy that silently no longer has the seam.
#
# Usage:  ./vendor_modbus.sh [git-ref]     (default: dev)
set -euo pipefail

REF="${1:-dev}"
DEST="$(cd "$(dirname "$0")" && pwd)/components/modbus"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Fetching esphome@${REF} ..."
curl -sSL "https://codeload.github.com/esphome/esphome/tar.gz/refs/heads/${REF}" \
  | tar xz -C "$TMP" --wildcards '*/esphome/components/modbus/*'

SRC="$(find "$TMP" -type d -path '*/esphome/components/modbus' | head -1)"
[ -d "$SRC" ] || { echo "modbus component not found in tarball" >&2; exit 1; }

rm -rf "$DEST"
mkdir -p "$(dirname "$DEST")"
cp -r "$SRC" "$DEST"

patch_line() {
  local file="$1" from="$2" to="$3"
  grep -qF -- "$to" "$file" && { echo "  already patched: $to"; return; }
  grep -qF -- "$from" "$file" || {
    echo "PATCH FAILED: '$from' not found in $file" >&2
    echo "Upstream changed the signature. Re-derive the patch before using this copy." >&2
    exit 1
  }
  # Replace only the declaration, which is the one occurrence in the header.
  python3 - "$file" "$from" "$to" <<'PY'
import sys
path, old, new = sys.argv[1], sys.argv[2], sys.argv[3]
s = open(path).read()
if s.count(old) != 1:
    sys.exit(f"PATCH FAILED: {old!r} appears {s.count(old)} times in {path}")
open(path, "w").write(s.replace(old, new))
PY
  echo "  patched: $to"
}

echo "Applying transport seam patch ..."
patch_line "$DEST/modbus.h" \
  "  void receive_bytes_();" \
  "  virtual void receive_bytes_();"
patch_line "$DEST/modbus.h" \
  "  bool send_frame_(const ModbusFrame &frame);" \
  "  virtual bool send_frame_(const ModbusFrame &frame);"
# timeout_() decides whether a partial response has gone stale, using the UART's
# rx_full_threshold. There is no UART behind a TCP hub, so this one has to be
# overridable too or it dereferences a null parent the moment a frame arrives.
patch_line "$DEST/modbus.h" \
  "  bool timeout_();" \
  "  virtual bool timeout_();"

echo
echo "Vendored to $DEST"
echo "Upstream ref: ${REF}"
