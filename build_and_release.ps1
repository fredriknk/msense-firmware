# Requires -Version 5.1
# -*- coding: utf-8 -*-

param(
    [string]$Target   = 'thumbv8m.main-none-eabihf',
    [string]$Profile  = 'release',
    [string]$BinName  = 'msense-firmware',
    [string[]]$Features = @('rev_1_3_2','rev_2_0','devboard')
)

$ErrorActionPreference = 'Stop'
function Die($m) { Write-Host "`n$m" -ForegroundColor Red; exit 1 }

# ───────────────────────── 0. Preconditions ──────────────────────────
if (git status --porcelain) { Die 'Working tree is dirty. Commit or stash first.' }

# pull version *once* so we know the tag name
function Get-Version {                                  # helper we’ll reuse
    (cargo metadata --format-version 1 --no-deps |
     ConvertFrom-Json).packages |
     Where-Object { $_.name -eq $BinName } |
     Select-Object -Expand version
}
$Ver = Get-Version
$Tag = "v$Ver"

# ────────── Does a GitHub release already exist for this tag? ──────────
$releaseExists = $false
try {
    & gh release view $Tag --json tagName *> $null       # no output streams
    if ($LASTEXITCODE -eq 0) { $releaseExists = $true }  # 0 ⇒ release found
} catch {
    # gh wrote "release not found" to its error stream ⇒ here on purpose
    $releaseExists = $false
}

if ($releaseExists) {
    Write-Host "`nA release for tag $Tag already exists."
    Write-Host '[O]verwrite assets  |  [B]ump patch version  |  [Q]uit'
    switch ((Read-Host 'Choose O / B / Q').ToUpper()) {
        'O' { Write-Host 'Will overwrite assets after build.' }
        'B' {
            Write-Host 'Running cargo-release patch...'
            cargo release patch --no-publish --no-tag --no-push --no-confirm --execute
            if ($LASTEXITCODE) { Die 'cargo-release patch failed.' }

            $Ver = Get-Version
            $Tag = "v$Ver"
            Write-Host "Version bumped to $Ver (tag $Tag)"
        }
        default { Die 'Aborted by user.' }
    }
}


# ─────────────────────── 1. Build all variants ───────────────────────
$BuildDir  = "target/$Target/$Profile"
$Artifacts = @()

foreach ($F in $Features) {
    Write-Host "`n==> Building with feature $F"
    cargo build --$Profile --target $Target --features $F
    if ($LASTEXITCODE) { Die "cargo build failed for $F" }

    $dst = "$BuildDir/${BinName}_${F}_${Ver}.elf"
    Copy-Item "$BuildDir/$BinName" $dst -Force
    $Artifacts += (Resolve-Path $dst).Path
}

# ─────────────────────── 2. Tag & push (once) ────────────────────────
if (-not (git tag -l $Tag)) {
    git tag -a $Tag -m "msense-firmware $Tag"
    git push origin $Tag
}

# ─────────────────────── 3. Release / upload ─────────────────────────
function MakeRelease([string[]]$files) {

    # ---- test for existing release safely -----------------------------
    $releaseExists = $false
    try {
        & gh release view $Tag --json tagName *> $null     # silence all streams
        if ($LASTEXITCODE -eq 0) { $releaseExists = $true }
    } catch {
        $releaseExists = $false                            # "release not found"
    }

    # ---- create or update ---------------------------------------------
    if ($releaseExists) {
        Write-Host "Uploading assets to existing release $Tag ..."
        gh release upload $Tag $files --clobber
    } else {
        Write-Host "Creating new release $Tag ..."
        gh release create $Tag $files `
            --title  ("msense-firmware " + $Tag) `
            --notes  ("Autobuild for "   + $Tag) `
            --latest
    }
}
MakeRelease $Artifacts
Write-Host ""
Write-Host ("Release {0} published with {1} binaries." -f $Tag, $Artifacts.Count)