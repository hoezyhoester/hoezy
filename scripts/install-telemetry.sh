#!/bin/bash
# Puts the motion-telemetry build of the game into an installed copy (Windows,
# Git Bash). Everything else of the installed copy - its game data, shaders,
# settings and shortcut - stays as the installer made it.
#
#   bash install-telemetry.sh ["<install directory>"]
#
# The install directory is the one the installer asked for, with bin/ and
# game/ in it; "~/Games/Burnout Revenge" (the installer's default) when left
# out. It installs what is missing to build (Visual Studio Build Tools, LLVM,
# CMake, Ninja, through winget), builds this branch from the installed copy's
# own game/default.xex, keeps the installed bin/ as bin_original/, puts the
# new burnout.exe and its libraries in bin/, and
# turns telemetry on in xerenge.conf. Run again to update; the backup is kept.
set -e

repo=https://github.com/hoezyhoester/hoezy
branch=motion-telemetry
work="$HOME/Xerenge-telemetry"

say() { printf '\n== %s\n' "$*"; }
fail() { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

case "$(uname -s)" in
    MINGW*|MSYS*) ;;
    *) fail "this script is for Windows, in Git Bash" ;;
esac

install=${1:-"$HOME/Games/Burnout Revenge"}
[ -f "$install/game/default.xex" ] ||
    fail "no game/default.xex in \"$install\". Run it with your install folder: bash install-telemetry.sh \"/c/path/to/Burnout Revenge\""
[ -f "$install/bin/burnout.exe" ] ||
    fail "no bin/burnout.exe in \"$install\" - install the game with the official installer first"
say "installed game: $install"

if tasklist 2>/dev/null | grep -qi '^burnout\.exe'; then
    fail "Burnout Revenge is running - close it first"
fi

# --- build tools --------------------------------------------------------------

# Where winget puts them, for this shell (its own PATH predates the installs).
export PATH="$PATH:/c/Program Files/LLVM/bin:/c/Program Files/CMake/bin:$(cygpath -u "$LOCALAPPDATA")/Microsoft/WinGet/Links"

winget_install() {
    say "installing $1 (Windows may ask for permission)"
    winget install --id "$1" -e --accept-package-agreements --accept-source-agreements "${@:2}" ||
        fail "could not install $1 - install it by hand and run this again"
}

command -v winget >/dev/null || fail "winget is missing (it comes with Windows 10/11's App Installer)"

vswhere="/c/Program Files (x86)/Microsoft Visual Studio/Installer/vswhere.exe"
has_msvc() {
    [ -x "$vswhere" ] && [ -n "$("$vswhere" -latest -products '*' \
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath)" ]
}
if ! has_msvc; then
    winget_install Microsoft.VisualStudio.2022.BuildTools --override \
        "--wait --passive --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
    has_msvc || fail "the Visual Studio C++ tools are still missing - install \"Desktop development with C++\" in the Visual Studio Installer"
fi
command -v clang >/dev/null || winget_install LLVM.LLVM
command -v cmake >/dev/null || winget_install Kitware.CMake
command -v ninja >/dev/null || winget_install Ninja-build.Ninja
# winget's portable packages (Ninja) land under WinGet/Packages, reached through
# links this shell may not follow: add their folders too.
for exe in "$(cygpath -u "$LOCALAPPDATA")"/Microsoft/WinGet/Packages/*/ninja.exe; do
    [ -f "$exe" ] && export PATH="$PATH:$(dirname "$exe")"
done
for tool in clang clang++ cmake ninja git; do
    command -v "$tool" >/dev/null || fail "$tool is still not found - close Git Bash, open it again and rerun this"
done
say "build tools are in place"

# --- the source ---------------------------------------------------------------

if [ -d "$work/.git" ]; then
    say "updating $work"
    git -C "$work" fetch --depth 1 origin "$branch"
    git -C "$work" checkout -q -B "$branch" FETCH_HEAD
else
    say "downloading the source to $work"
    git clone --depth 1 -b "$branch" "$repo" "$work"
fi
cd "$work"
git submodule update --init --depth 1 rexglue/thirdparty/etcpak

# --- the build ----------------------------------------------------------------

export CC=clang CXX=clang++
say "building the SDK (the longest step; it is only slow the first time)"
./scripts/setup-sdk.sh
say "building the renderer and shader translator"
./scripts/setup-deps.sh
say "building the game from your disc's default.xex"
./scripts/build.sh --game "$install/game"
[ -f rexglue/build/burnout.exe ] || fail "the build made no burnout.exe"

# --- into the installed copy ----------------------------------------------------

bin="$install/bin"
if [ ! -d "$install/bin_original" ]; then
    cp -r "$bin" "$install/bin_original"
    say "kept the original game as bin_original"
fi
cp rexglue/build/burnout.exe "$bin/burnout.exe"
find rexglue/build rexglue-sdk/out/win-amd64 -maxdepth 2 -name '*.dll' -not -path '*/Debug/*' \
    -exec cp {} "$bin/" \;
say "new burnout.exe and its libraries are in $bin"

conf="$install/xerenge.conf"
touch "$conf"
if grep -q '^[[:space:]]*telemetry[[:space:]]*=' "$conf"; then
    sed -i 's/^[[:space:]]*telemetry[[:space:]]*=.*/telemetry = on/' "$conf"
else
    printf '\ntelemetry = on\n' >> "$conf"
fi
say "telemetry is on in xerenge.conf"

cat <<'EOF'

Done. Start SpaceMonkey, press Burnout Revenge > Initialize!, then start the
game with its usual shortcut and begin a race.

To go back to the original game: in the install folder, delete bin and
rename bin_original to bin.
EOF
