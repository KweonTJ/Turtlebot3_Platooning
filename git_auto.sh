#!/bin/bash
set -e

cd "$(dirname "$0")" || exit 1

echo "=== Git Auto Sync Start ==="

BRANCH=$(git branch --show-current)
if [ -z "$BRANCH" ]; then
    echo "Detached HEAD state; aborting auto sync."
    exit 1
fi

echo "[1] Pull latest changes..."
if git ls-remote --exit-code --heads origin "$BRANCH" >/dev/null 2>&1; then
    git pull origin "$BRANCH" --rebase
else
    echo "[1] Remote branch origin/$BRANCH not found; skipping pull."
fi

echo "[2] Add changes..."
git add .

if git diff --cached --quiet; then
    echo "[3] No changes to commit."
else
    NOW=$(date "+%Y-%m-%d %H:%M:%S")
    COMMIT_MESSAGE=${1:-"Auto update: $NOW"}
    echo "[3] Commit changes..."
    git commit -m "$COMMIT_MESSAGE"

    echo "[4] Push changes..."
    git push origin "$BRANCH"
fi

echo "=== Git Auto Sync Done ==="
