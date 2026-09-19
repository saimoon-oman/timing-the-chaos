#!/usr/bin/env bash
# Move the superseded v3 figures out of the way of the current v4 set.
#
# The v3 PNGs live directly in analysis/figures/ (plus figures/sp/ and
# figures/sll/).  The current figures are in analysis/figures/v4/.  Nothing
# breaks if you skip this; it only makes the folder easier to read.
#
#   bash tools/archive_v3_figures.sh          # move them
#   bash tools/archive_v3_figures.sh --dry-run
set -euo pipefail

cd "$(dirname "$0")/.."
FIG=analysis/figures
DEST=$FIG/archive-v3
DRY=${1:-}

[ -d "$FIG" ] || { echo "no $FIG -- run this from the repository"; exit 1; }

move() {
  local src=$1 dst=$2
  if [ "$DRY" = "--dry-run" ]; then
    echo "would move  $src -> $dst"
    return
  fi
  mkdir -p "$(dirname "$dst")"
  if git ls-files --error-unmatch "$src" >/dev/null 2>&1; then
    git mv -f "$src" "$dst"
  else
    mv -f "$src" "$dst"
  fi
  echo "moved  $src -> $dst"
}

shopt -s nullglob
for f in "$FIG"/*.png "$FIG"/*.txt; do
  move "$f" "$DEST/$(basename "$f")"
done
for d in sp sll; do
  [ -d "$FIG/$d" ] || continue
  for f in "$FIG/$d"/*; do
    move "$f" "$DEST/$d/$(basename "$f")"
  done
done

# --- the project site ------------------------------------------------------
# docs/assets/figures/ also holds the superseded v3 PNGs.  The site references
# only the ten below; everything else there is v3 and can be archived.
SITE=docs/assets/figures
KEEP="attack_params.png convergence.png defence.png detector.png \
dose_response.png mix_sensitivity.png strip_ablation.png threat_model.png \
volume_mix_grid.png volume_sensitivity.png"

if [ -d "$SITE" ]; then
  for f in "$SITE"/*.png; do
    b=$(basename "$f")
    case " $KEEP " in *" $b "*) continue ;; esac
    move "$f" "$SITE/archive-v3/$b"
  done
fi

echo
echo "done.  current figures remain in $FIG/v4/ and $SITE/"
[ "$DRY" = "--dry-run" ] || echo "review with 'git status', then commit."
