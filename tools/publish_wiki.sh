#!/usr/bin/env bash
# Publish wiki/ to the repository's GitHub wiki.
#
# GitHub keeps the wiki in a SEPARATE git repository at
#   https://github.com/<owner>/<repo>.wiki.git
# which does not exist until you create at least one page through the web UI.
#
#   1. Go to https://github.com/saimoon-oman/timing-the-chaos/wiki
#   2. Click "Create the first page", type anything, Save.
#   3. Run this script.
#
#   bash tools/publish_wiki.sh                    # push wiki/ as-is
#   bash tools/publish_wiki.sh "message"          # with a commit message
set -euo pipefail

cd "$(dirname "$0")/.."
SRC=wiki
MSG=${1:-"Sync wiki from repository wiki/ (v4)"}

[ -d "$SRC" ] || { echo "no $SRC/ -- run this from the repository"; exit 1; }

ORIGIN=$(git config --get remote.origin.url)
WIKI=${ORIGIN%.git}.wiki.git
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "wiki remote: $WIKI"
if ! git clone --quiet "$WIKI" "$TMP/wiki" 2>/dev/null; then
  cat <<'EOF'
Could not clone the wiki repository.

Almost always this means the wiki has never been initialised. Open
  https://github.com/saimoon-oman/timing-the-chaos/wiki
create the first page through the web UI, then run this script again.
EOF
  exit 1
fi

# README.md in wiki/ documents the folder itself; it is not a wiki page.
for f in "$SRC"/*.md; do
  [ "$(basename "$f")" = "README.md" ] && continue
  cp "$f" "$TMP/wiki/"
done

cd "$TMP/wiki"
if git diff --quiet && git diff --cached --quiet && [ -z "$(git status --porcelain)" ]; then
  echo "wiki already up to date."
  exit 0
fi
git add -A
git commit -qm "$MSG"
git push -q origin HEAD
echo "pushed $(ls *.md | wc -l) pages to the wiki."
