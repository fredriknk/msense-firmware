param(
    [string]$Target   = 'thumbv8m.main-none-eabihf',
    [string]$Profile  = 'release',
    [string]$BinName  = 'msense-firmware',
    [string[]]$Features = @('rev_1_3_2','rev_2_0','devboard')
)

$ErrorActionPreference = 'Stop'
function Die($m) { Write-Host "`n$m" -ForegroundColor Red; exit 1 }

# ────────── Does a GitHub release already exist for this tag? ──────────
$releaseExists = $false
& gh release view $Tag >$null 2>$null   # silence stdout + stderr

if ($LASTEXITCODE -eq 0) {              # exit-code 0 ⇒ release found
    $releaseExists = $true
}

if ($releaseExists) {
    Write-Host "`nA release for tag $Tag already exists."
    Write-Host '[O]verwrite assets  |  [B]ump patch version  |  [Q]uit'
    switch ((Read-Host 'Choose O / B / Q').ToUpper()) {
        'O' { Write-Host 'Will overwrite assets after build.' }
        'B' {
            Write-Host 'Running cargo-release patch …'
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
    Copy-Item "$BuildDir/$BinName.elf" $dst -Force
    $Artifacts += (Resolve-Path $dst).Path
}

# ─────────────────────── 2. Tag & push (once) ────────────────────────
if (-not (git tag -l $Tag)) {
    git tag -a $Tag -m "msense-firmware $Tag"
    git push origin $Tag
}

# ─────────────────────── 3. Release / upload ─────────────────────────
function MakeRelease($files) {
    if (gh release view $Tag 2>$null) {
        gh release upload $Tag $files --clobber
    } else {
        gh release create $Tag $files `
            --title  ("msense-firmware " + $Tag) `
            --notes  ("Autobuild for "   + $Tag) `
            --latest
    }
}
MakeRelease $Artifacts

Write-Host "`n  Release $Tag published with $($Artifacts.Count) binaries."
