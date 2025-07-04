param(
    [string]$Target   = 'thumbv8m.main-none-eabihf',
    [string]$Profile  = 'release',
    [string]$BinName  = 'msense-firmware',
    [string[]]$Features = @('rev_1_3_2','rev_2_0','devboard')
)

$ErrorActionPreference = 'Stop'
function Die([string]$Msg) { Write-Host "`n$Msg" -ForegroundColor Red; exit 1 }

# 0.  Require clean tree ------------------------------------------------
if (git status --porcelain) { Die 'Working tree is dirty. Commit or stash first.' }

# 1.  Load crate version  ----------------------------------------------
function Get-Version {
    (cargo metadata --format-version 1 --no-deps |
        ConvertFrom-Json).packages |
        Where-Object { $_.name -eq $BinName } |
        Select-Object -Expand version
}

$Ver = Get-Version
$Tag = "v$Ver"

# 2.  Decide what to do with the tag/release ---------------------------
if (gh release view $Tag 2>$null) {
    Write-Host "`nA GitHub release for tag $Tag already exists."
    Write-Host '[O]verwrite assets  |  [B]ump patch version and continue  |  [Q]uit'
    $choice = (Read-Host 'Choose O / B / Q').ToUpper()

    switch ($choice) {
        'O' {
            $ReleaseAction = 'overwrite'    # remember for later
        }
        'B' {
            Write-Host "`nRunning: cargo release patch --no-publish --no-tag --no-push --no-confirm --execute"
            cargo release patch --no-publish --no-tag --no-push --no-confirm --execute
            if ($LASTEXITCODE -ne 0) { Die 'cargo release patch failed.' }

            # refresh version & tag after bump
            $Ver = Get-Version
            $Tag = "v$Ver"
            $ReleaseAction = 'create'       # new tag, new release
        }
        'Q' { Die 'Aborted by user.' }
        default { Die 'Invalid choice.' }
    }
} else {
    $ReleaseAction = 'create'               # fresh tag
}

Write-Host "`nUsing tag $Tag  (version $Ver)"


# 3.  Build every feature
$BuildDir  = "target/$Target/$Profile"
$Artifacts = @()

foreach ($F in $Features) {
    Write-Host ''
    Write-Host ("==> Building with feature {0}" -f $F)

    cargo build --$Profile --target $Target --features $F
    if ($LASTEXITCODE -ne 0) { Die ("cargo build failed for {0}" -f $F) }

    $Dst = "$BuildDir/$BinName" + "_$F" + "_$Ver.elf"
    Copy-Item "$BuildDir/$BinName" $Dst -Force
    $Artifacts += $Dst
    Write-Host ("Built  {0}" -f $Dst)
}

# 4.  Git tag + push (only if it doesn't exist)
$Tag = "v$Ver"
if (-not (git tag -l $Tag)) {
    git tag -a $Tag -m ("msense-firmware {0}" -f $Tag)
    git push origin $Tag
}

# 5.  GitHub CLI: build args entirely with single-quoted literals
$ghArgs = @(
    'release','create', $Tag,
    '--title', ('msense-firmware ' + $Tag),
    '--notes', ('Autobuild for '   + $Tag),
    '--latest'
) + $Artifacts

Write-Host ''
Write-Host ('Invoking: gh ' + ($ghArgs -join ' '))

& gh @ghArgs      # call-operator (&) + argument array
