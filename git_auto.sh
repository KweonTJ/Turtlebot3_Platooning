#!/bin/bash

cd "$(dirname "$0")" || exit 1

echo "=== Git Auto Sync Start ==="

echo "[1] Pull latest changes..."
git pull origin main --rebase

echo "[2] Add changes..."
git add .

if git diff --cached --quiet; then
    echo "[3] No changes to commit."
else
    NOW=$(date "+%Y-%m-%d %H:%M:%S")
    echo "[3] Commit changes..."
    git commit -m "Auto update: $NOW"

    echo "[4] Push changes..."
    git push origin main
fi

echo "=== Git Auto Sync Done ==="
