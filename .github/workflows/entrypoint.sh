#!/bin/bash

sh -c "git config --global --add safe.directory $PWD"

set -eu

REPO_FULLNAME=$(jq -r ".repository.full_name" "$GITHUB_EVENT_PATH")

echo "## Initializing git repo..."
git init

echo "### Adding git remote..."
git remote add origin https://x-access-token:$GITHUB_TOKEN@github.com/$REPO_FULLNAME.git
# git remote set-url origin https://x-access-token:$GITHUB_TOKEN@github.com/$REPO_FULLNAME.git
