#!/usr/bin/env bash
set -euo pipefail

Target="thumbv8m.main-none-eabihf"
Profile="release"
BinName="msense-firmware"
Features=("rev_1_3_2" "rev_2_0" "devboard")

usage() {
  echo "Usage: $0 [-t target] [-p profile] [-b binname] [-f feature1,feature2,...]" >&2
  exit 1
}

while getopts "t:p:b:f:" opt; do
  case $opt in
    t) Target=$OPTARG ;;
    p) Profile=$OPTARG ;;
    b) BinName=$OPTARG ;;
    f) IFS=',' read -r -a Features <<<"$OPTARG" ;;
    *) usage ;;
  esac
done

if [[ -n $(git status --porcelain) ]]; then
  echo "Working tree is dirty. Commit or stash first." >&2
  exit 1
fi

get_version() {
  cargo metadata --format-version 1 --no-deps | \
    jq -r --arg name "$BinName" '.packages[] | select(.name==$name) | .version'
}

Ver=$(get_version)
Tag="v$Ver"

if gh release view "$Tag" >/dev/null 2>&1; then
  echo "A release for tag $Tag already exists."
  echo "[O]verwrite assets  |  [B]ump patch version  |  [Q]uit"
  read -r -p "Choose O / B / Q: " choice
  case "${choice^^}" in
    O) echo "Will overwrite assets after build." ;;
    B)
      echo "Running cargo-release patch..."
      cargo release patch --no-publish --no-tag --no-push --no-confirm --execute
      Ver=$(get_version)
      Tag="v$Ver"
      echo "Version bumped to $Ver (tag $Tag)"
      ;;
    *) echo "Aborted by user." ; exit 1 ;;
  esac
fi

BuildDir="target/$Target/$Profile"
mkdir -p "$BuildDir"
Artifacts=()

for F in "${Features[@]}"; do
  echo "==> Building with feature $F"
  cargo build --$Profile --target "$Target" --features "$F"
  dst="$BuildDir/${BinName}_${F}_${Ver}.elf"
  cp "$BuildDir/$BinName" "$dst"
  Artifacts+=("$dst")
done

if ! git tag -l "$Tag" >/dev/null 2>&1; then
  git tag -a "$Tag" -m "msense-firmware $Tag"
  git push origin "$Tag"
fi

if gh release view "$Tag" >/dev/null 2>&1; then
  echo "Uploading assets to existing release $Tag ..."
  gh release upload "$Tag" "${Artifacts[@]}" --clobber
else
  echo "Creating new release $Tag ..."
  gh release create "$Tag" "${Artifacts[@]}" \
    --title "msense-firmware $Tag" \
    --notes "Autobuild for $Tag" \
    --latest
fi

echo "Release $Tag published with ${#Artifacts[@]} binaries."
