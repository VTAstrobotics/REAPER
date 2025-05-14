#!/bin/bash

sh -c "git config --global --add safe.directory $PWD"

set -eu

REPO_FULLNAME=$(jq -r ".repository.full_name" "$GITHUB_EVENT_PATH")

echo "## Initializing git repo..."
git config --global init.defaultBranch main
git init

echo "## Perms for new git stuff"
chmod -R 777 .

echo "### Adding git remote..."
git remote add origin https://x-access-token:$GITHUB_TOKEN@github.com/$REPO_FULLNAME.git
# git remote set-url origin https://x-access-token:$GITHUB_TOKEN@github.com/$REPO_FULLNAME.git
