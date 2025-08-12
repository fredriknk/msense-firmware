# Msense NRF9160

Firmware for the msense NRF9160, a project to build a low-cost, open-source methane sensor using the Nordic NRF9160 and Figaro TGS8410 low power methane sensor.

The program communicates over LTE-m with CBOR encoded data through mTLS TCP steams to a self hosted server.

Before building, add a `.env` with your endpoint `HOST_ADDRESS=<your_host_address>` variable.

Run `build_and_release.ps1` to release to GitHub. For Linux, use `./build_and_release.sh`.

## Environment setup (Linux)

<details><summary>Click to show details</summary>

1. Install Rust and Cargo via [rustup](https://rustup.rs/).
2. Add the target:
   ```bash
   rustup target add thumbv8m.main-none-eabihf
   ```
3. Install the LLVM tools component:
   ```bash
   rustup component add llvm-tools
   ```
4. Install an ARM cross-compiler (Debian/Ubuntu):
   ```bash
   sudo apt-get update
   sudo apt-get install gcc-arm-none-eabi
   ```
5. Build the firmware using the `devboard` feature:
   ```bash
   cargo build --features devboard
   ```
</details>

## Building on Windows-amd64 (Windows OS)

<details><summary>Click to show details</summary>

###  ✅ Compile for `windows-amd64`

> Compiling msense_firmware with windows OS requires **Administrator** privilege!

⚠**Tested on Windows 10 build 22H2(22621.963)**

1. Install LLVM 👉 [Download Here](https://github.com/llvm/llvm-project/releases) *look for something like `LLVM-X.Y.Z-win64.exe`* or 
```bash
choco install llvm
```

2. Install `MYSYS2` 👉 Follow instructions on [their website](https://www.msys2.org/)
or 
```bash
choco install msys2
```

3. When `MYSYS2` is installed it opens a shell. Install GCC via that shell

	```bash
	pacman -S mingw-w64-i686-gcc
	pacman -S mingw-w64-x86_64-gcc
	```

4. Install `patch` GNU Util

	Go to GNUWin32 page for [*patch*](http://gnuwin32.sourceforge.net/packages/patch.htm) and
	extract the [*patch*](http://gnuwin32.sourceforge.net/downlinks/patch-bin-zip.php)
	binaries onto your drive. 
	
	e.g. "C:\patch"

	> For some bizzarre reasons, **patch.exe needs elevated privileges** to be invoked during compilation

5. Add `gcc` and `patch` binary path to environment

	> Setting the environment variables for the entire system requires an **elevated ⚠** shell.

	```powershell
	[System.Environment]::SetEnvironmentVariable(
		"PATH", 
		[System.Environment]::GetEnvironmentVariable("PATH", "Machine") + 
		";" +
		"C:\msys64\mingw64\bin;" +
		"C:\msys64\mingw32\bin;" +
		"C:\patch\bin;",
		"Machine"
	)
	```

	Or do it manually via `sysdm.cpl`.

6. Run cargo in an **elevated ⚠** shell

    Running `cargo build` in an **elevated shell** will now build the `nrfxlib` in Windows OS.
</details>

