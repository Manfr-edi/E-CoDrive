#!/usr/bin/env bash

set -euo pipefail

UTLEXUS_ASSET_URL="https://github.com/UT-ADL/carla_lexus/releases/download/v0.9.15/utlexus.tar.gz"
UTLEXUS_ASSET_FILENAME="utlexus.tar.gz"
UTLEXUS_ASSET_MARKER=".utlexus_asset_imported"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

usage() {
    cat <<'EOF'
Usage:
  ./scripts/setup_carla.sh [<CARLA_DIR> [0.9.13|0.9.15]]

What it does:
  - installs the project-specific SUMO/CARLA files into a vanilla CARLA folder
  - writes the version-specific `vtypes.json`
  - writes the version-specific `egovtype.xml`
  - patches the runtime-critical `bridge_helper.py`
  - patches the runtime-critical `sumo_simulation.py`
  - patches `create_sumo_vtypes.py` so generated SUMO vTypes keep nested params
  - replaces any remaining `bak_carlavtypes.rou.xml` references with `carlavtypes.rou.xml`
  - downloads the UT Lexus CARLA asset archive into `Import/` when needed
  - runs `./ImportAssets.sh` when asset archives are present in `Import/`
  - removes imported `*.tar.gz` asset archives from `Import/` after a successful import
  - when `CARLA 0.9.13` is part of the setup, builds/starts the
    `autoware_mini` Docker container if Docker Compose is available

Notes:
  - `custom_<Town>.sumocfg` files are not required beforehand; the dashboard generates them.
  - the script creates backups under `<CARLA_DIR>/.customcosim-backups/<timestamp>/`.
  - if `<CARLA_DIR>` is omitted, the script auto-detects `carla/CARLA_*`
    installations first, then the legacy root-level `CARLA_0.9.13` /
    `CARLA_0.9.15` folders inside this repository.
  - shell environment variables such as `SUMO_HOME` and `CARLA_PYTHON*`
    are not persisted by this script; set them manually only when your
    local runtime does not match the documented defaults.
EOF
}

die() {
    echo "ERROR: $*" >&2
    exit 1
}

info() {
    echo "[setup_carla] $*"
}

run_cmd() {
    info "Running: $*"
    "$@"
}

require_dir() {
    local path="$1"
    [[ -d "$path" ]] || die "Directory not found: $path"
}

require_file() {
    local path="$1"
    [[ -f "$path" ]] || die "File not found: $path"
}

is_carla_install_dir() {
    local path="$1"
    [[ -d "$path" && -f "$path/CarlaUE4.sh" && -d "$path/Co-Simulation/Sumo" ]]
}

resolve_carla_dir() {
    local input_dir="$1"
    local requested_version="${2:-}"
    local resolved_input
    resolved_input="$(cd -- "$input_dir" && pwd)"

    if is_carla_install_dir "$resolved_input"; then
        echo "$resolved_input"
        return 0
    fi

    local nested_candidates=()
    local nested_dir
    for nested_dir in \
        "$resolved_input/CARLA_0.9.13" \
        "$resolved_input/CARLA_0.9.15"
    do
        if is_carla_install_dir "$nested_dir"; then
            nested_candidates+=("$nested_dir")
        fi
    done

    if [[ ${#nested_candidates[@]} -eq 0 ]]; then
        echo "$resolved_input"
        return 0
    fi

    if [[ -n "$requested_version" ]]; then
        local version_match="$resolved_input/CARLA_$requested_version"
        if is_carla_install_dir "$version_match"; then
            echo "$version_match"
            return 0
        fi
        die "Requested CARLA version $requested_version not found under $resolved_input."
    fi

    if [[ ${#nested_candidates[@]} -gt 1 ]]; then
        die "Multiple CARLA installations found under $resolved_input. Pass the exact folder or specify the version explicitly."
    fi

    echo "${nested_candidates[0]}"
}

detect_version() {
    local carla_dir="$1"
    local explicit_version="${2:-}"

    if [[ -n "$explicit_version" ]]; then
        case "$explicit_version" in
            0.9.13|0.9.15)
                echo "$explicit_version"
                return 0
                ;;
            *)
                die "Unsupported CARLA version: $explicit_version"
                ;;
        esac
    fi

    local base_name
    base_name="$(basename "$carla_dir")"
    case "$base_name" in
        *0.9.13*)
            echo "0.9.13"
            return 0
            ;;
        *0.9.15*)
            echo "0.9.15"
            return 0
            ;;
    esac

    local version_file="$carla_dir/VERSION"
    if [[ -f "$version_file" ]]; then
        if grep -q "0.9.13" "$version_file"; then
            echo "0.9.13"
            return 0
        fi
        if grep -q "0.9.15" "$version_file"; then
            echo "0.9.15"
            return 0
        fi
    fi

    local dist_dir="$carla_dir/PythonAPI/carla/dist"
    if [[ -d "$dist_dir" ]]; then
        if compgen -G "$dist_dir/carla-*0.9.13*" >/dev/null; then
            echo "0.9.13"
            return 0
        fi
        if compgen -G "$dist_dir/carla-*0.9.15*" >/dev/null; then
            echo "0.9.15"
            return 0
        fi
    fi

    die "Could not infer CARLA version from '$carla_dir'. Pass 0.9.13 or 0.9.15 explicitly."
}

backup_file() {
    local target="$1"
    if [[ ! -f "$target" ]]; then
        return 0
    fi

    local relative_path="${target#$CARLA_DIR/}"
    local backup_target="$BACKUP_DIR/$relative_path"
    mkdir -p "$(dirname "$backup_target")"
    cp -a "$target" "$backup_target"
}

install_file() {
    local source="$1"
    local target="$2"
    local label="$3"

    require_file "$source"
    mkdir -p "$(dirname "$target")"
    backup_file "$target"
    cp "$source" "$target"
    info "Installed $label -> $target"
}

replace_carlavtypes_reference() {
    local target_file="$1"
    if [[ ! -f "$target_file" ]]; then
        return 0
    fi

    python3 - "$target_file" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")
updated = text.replace("bak_carlavtypes.rou.xml", "carlavtypes.rou.xml")
if updated != text:
    path.write_text(updated, encoding="utf-8")
PY
}

verify_contains() {
    local target="$1"
    local pattern="$2"
    local label="$3"
    if ! grep -q "$pattern" "$target"; then
        die "Verification failed for $label: pattern '$pattern' not found in $target"
    fi
}

download_file() {
    local url="$1"
    local destination="$2"

    if command -v curl >/dev/null 2>&1; then
        curl -L --fail --retry 3 --output "$destination" "$url"
        return 0
    fi

    if command -v wget >/dev/null 2>&1; then
        wget -O "$destination" "$url"
        return 0
    fi

    python3 - "$url" "$destination" <<'PY'
import shutil
import sys
import urllib.request

url, destination = sys.argv[1], sys.argv[2]
with urllib.request.urlopen(url) as response, open(destination, "wb") as output:
    shutil.copyfileobj(response, output)
PY
}

ensure_utlexus_asset_archive() {
    local import_dir="$1"
    local archive_path="$import_dir/$UTLEXUS_ASSET_FILENAME"
    local marker_path="$import_dir/$UTLEXUS_ASSET_MARKER"
    local temp_path="$archive_path.part"

    mkdir -p "$import_dir"

    if [[ -f "$archive_path" ]]; then
        info "Using existing UT Lexus asset archive at $archive_path"
        return 0
    fi

    if [[ -f "$marker_path" ]]; then
        info "UT Lexus asset already imported for $import_dir; skipping download."
        return 0
    fi

    info "Downloading UT Lexus asset archive into $archive_path"
    rm -f -- "$temp_path"
    if ! download_file "$UTLEXUS_ASSET_URL" "$temp_path"; then
        rm -f -- "$temp_path"
        die "Could not download $UTLEXUS_ASSET_URL"
    fi
    mv -- "$temp_path" "$archive_path"
}

utlexus_asset_installed() {
    local carla_dir="$1"
    [[ -f "$carla_dir/CarlaUE4/Content/Carla/Blueprints/Vehicles/Lexus/BP_Lexus.uasset" ]]
}

import_carla_assets() {
    local carla_dir="$1"
    local import_script="$carla_dir/ImportAssets.sh"
    local import_dir="$carla_dir/Import"
    local marker_path="$import_dir/$UTLEXUS_ASSET_MARKER"
    local archives=()
    local archive
    local imported_utlexus=0
    local import_exit_code=0

    if [[ ! -f "$import_script" ]]; then
        info "Skipping asset import: $import_script not found."
        return 0
    fi

    if [[ ! -d "$import_dir" ]]; then
        mkdir -p "$import_dir"
        info "Created asset import directory $import_dir"
    fi

    ensure_utlexus_asset_archive "$import_dir"

    while IFS= read -r -d '' archive; do
        archives+=("$archive")
    done < <(find "$import_dir" -maxdepth 1 -type f -name '*.tar.gz' -print0)

    if [[ ${#archives[@]} -eq 0 ]]; then
        info "No asset archives found in $import_dir; skipping ImportAssets.sh."
        return 0
    fi

    info "Running ImportAssets.sh for ${#archives[@]} asset archive(s)."
    set +e
    (
        cd -- "$carla_dir"
        bash ./ImportAssets.sh
    )
    import_exit_code=$?
    set -e

    if [[ $import_exit_code -ne 0 ]]; then
        if utlexus_asset_installed "$carla_dir"; then
            info "ImportAssets.sh exited with code $import_exit_code, but the UT Lexus asset is already present. Continuing with cleanup."
        else
            die "ImportAssets.sh failed with exit code $import_exit_code before the UT Lexus asset became available."
        fi
    fi

    for archive in "${archives[@]}"; do
        if [[ "$(basename "$archive")" == "$UTLEXUS_ASSET_FILENAME" ]]; then
            imported_utlexus=1
        fi
        if [[ -f "$archive" ]]; then
            rm -f -- "$archive"
            info "Removed imported asset archive $archive"
        fi
    done

    if [[ $imported_utlexus -eq 1 ]]; then
        : > "$marker_path"
        info "Marked UT Lexus asset as imported in $marker_path"
    fi
}

autoware_compose_cmd=()

resolve_autoware_compose_command() {
    if command -v docker >/dev/null 2>&1; then
        if docker compose version >/dev/null 2>&1; then
            autoware_compose_cmd=(docker compose)
            return 0
        fi
    fi

    if command -v docker-compose >/dev/null 2>&1; then
        autoware_compose_cmd=(docker-compose)
        return 0
    fi

    return 1
}

setup_autoware_docker() {
    local compose_dir="$REPO_ROOT/autoware_mini_docker_compose"
    local compose_file="$compose_dir/docker-compose.yml"
    local container_name="autoware_mini"
    local docker_binary=""
    local container_exists=1
    local container_running=1
    local process_count

    if [[ ! -d "$compose_dir" || ! -f "$compose_file" ]]; then
        info "Skipping Autoware Docker setup: $compose_file not found."
        return 0
    fi

    if ! resolve_autoware_compose_command; then
        info "Skipping Autoware Docker setup: Docker Compose is not available."
        info "Manual fallback: from $compose_dir run \`docker compose up -d --build autoware_mini\`."
        return 0
    fi

    process_count="${PROCESS_COUNT:-}"
    if [[ -z "$process_count" ]] && command -v nproc >/dev/null 2>&1; then
        process_count="$(nproc)"
    fi

    docker_binary="$(command -v docker || true)"
    if [[ -n "$docker_binary" ]]; then
        if "$docker_binary" container inspect "$container_name" >/dev/null 2>&1; then
            container_exists=0
            if [[ "$("$docker_binary" container inspect --format '{{.State.Running}}' "$container_name" 2>/dev/null)" == "true" ]]; then
                container_running=0
            fi
        fi
    fi

    if [[ $container_running -eq 0 ]]; then
        info "Autoware Docker container '$container_name' is already running."
        return 0
    fi

    if [[ $container_exists -eq 0 && -n "$docker_binary" ]]; then
        info "Starting existing Autoware Docker container '$container_name'."
        run_cmd "$docker_binary" start "$container_name" >/dev/null
        info "Autoware Docker container is ready."
        return 0
    fi

    info "Building and starting Autoware Docker container from $compose_dir"
    (
        cd -- "$compose_dir"
        if [[ -n "$process_count" ]]; then
            export PROCESS_COUNT="$process_count"
        fi
        run_cmd "${autoware_compose_cmd[@]}" up -d --build "$container_name"
    )
    info "Autoware Docker container is ready."
}

bootstrap_carla_dir() {
    local input_dir="$1"
    local requested_version="${2:-}"

    CARLA_DIR="$(resolve_carla_dir "$input_dir" "$requested_version")"
    VERSION="$(detect_version "$CARLA_DIR" "$requested_version")"
    TEMPLATE_DIR="$REPO_ROOT/bootstrap_templates/$VERSION"
    COMMON_TEMPLATE_DIR="$REPO_ROOT/bootstrap_templates/common"
    TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
    BACKUP_DIR="$CARLA_DIR/.customcosim-backups/$TIMESTAMP"

    SUMO_DIR="$CARLA_DIR/Co-Simulation/Sumo"
    EXAMPLES_DIR="$SUMO_DIR/examples"
    DATA_DIR="$SUMO_DIR/data"
    SUMO_INTEGRATION_DIR="$SUMO_DIR/sumo_integration"
    UTIL_DIR="$SUMO_DIR/util"

    VTYPES_JSON="$DATA_DIR/vtypes.json"
    EGO_VTYPE_XML="$EXAMPLES_DIR/egovtype.xml"
    CARLAVTYPES_XML="$EXAMPLES_DIR/carlavtypes.rou.xml"
    VIEWSETTINGS_XML="$EXAMPLES_DIR/viewsettings.xml"
    BRIDGE_HELPER_PY="$SUMO_INTEGRATION_DIR/bridge_helper.py"
    SUMO_SIMULATION_PY="$SUMO_INTEGRATION_DIR/sumo_simulation.py"
    CREATE_SUMO_VTYPES_PY="$UTIL_DIR/create_sumo_vtypes.py"

    require_dir "$CARLA_DIR"
    require_dir "$SUMO_DIR"
    require_dir "$EXAMPLES_DIR"
    require_dir "$DATA_DIR"
    require_dir "$SUMO_INTEGRATION_DIR"
    require_dir "$UTIL_DIR"
    require_dir "$EXAMPLES_DIR/net"
    require_file "$VTYPES_JSON"
    require_file "$CARLAVTYPES_XML"
    require_file "$VIEWSETTINGS_XML"
    require_file "$BRIDGE_HELPER_PY"
    require_file "$SUMO_SIMULATION_PY"
    require_file "$CREATE_SUMO_VTYPES_PY"
    require_file "$TEMPLATE_DIR/vtypes.json"
    require_file "$TEMPLATE_DIR/egovtype.xml"
    require_file "$COMMON_TEMPLATE_DIR/bridge_helper.py"
    require_file "$COMMON_TEMPLATE_DIR/sumo_simulation.py"
    require_file "$COMMON_TEMPLATE_DIR/create_sumo_vtypes.py"

    mkdir -p "$BACKUP_DIR"
    info "Target CARLA dir: $CARLA_DIR"
    info "Detected version: $VERSION"
    info "Backup dir: $BACKUP_DIR"

    install_file "$TEMPLATE_DIR/vtypes.json" "$VTYPES_JSON" "vtypes.json"
    install_file "$TEMPLATE_DIR/egovtype.xml" "$EGO_VTYPE_XML" "egovtype.xml"
    install_file "$COMMON_TEMPLATE_DIR/bridge_helper.py" "$BRIDGE_HELPER_PY" "bridge_helper.py"
    install_file "$COMMON_TEMPLATE_DIR/sumo_simulation.py" "$SUMO_SIMULATION_PY" "sumo_simulation.py"
    install_file "$COMMON_TEMPLATE_DIR/create_sumo_vtypes.py" "$CREATE_SUMO_VTYPES_PY" "create_sumo_vtypes.py"

    for optional_file in \
        "$SUMO_DIR/routes.rou.xml" \
        "$EXAMPLES_DIR/routes.rou.xml" \
        "$EXAMPLES_DIR/trips.trips.xml"
    do
        backup_file "$optional_file"
        replace_carlavtypes_reference "$optional_file"
    done

    while IFS= read -r -d '' custom_cfg; do
        backup_file "$custom_cfg"
        replace_carlavtypes_reference "$custom_cfg"
        info "Patched carlavtypes reference in $custom_cfg"
    done < <(find "$EXAMPLES_DIR" -maxdepth 1 -type f -name 'custom_*.sumocfg' -print0)

    verify_contains "$VTYPES_JSON" '"vehicle.lexus.utlexus"' "vtypes.json"
    verify_contains "$VTYPES_JSON" '"device.battery.capacity"' "vtypes.json"
    verify_contains "$EGO_VTYPE_XML" 'carla.blueprint' "egovtype.xml"
    verify_contains "$BRIDGE_HELPER_PY" 'setEmissionClass' "bridge_helper.py"
    verify_contains "$SUMO_SIMULATION_PY" 'sumolib.net.readNet' "sumo_simulation.py"
    verify_contains "$CREATE_SUMO_VTYPES_PY" '_split_vtype_specs' "create_sumo_vtypes.py"

    python3 -m py_compile "$BRIDGE_HELPER_PY" "$SUMO_SIMULATION_PY" "$CREATE_SUMO_VTYPES_PY" >/dev/null

    import_carla_assets "$CARLA_DIR"

    if [[ "$VERSION" == "0.9.13" ]]; then
        setup_autoware_docker
    fi

    info "Bootstrap completed for $CARLA_DIR."
    info "The dashboard will generate custom_Town*.sumocfg on demand."
}

if [[ $# -ge 1 ]]; then
    case "$1" in
        -h|--help)
            usage
            exit 0
            ;;
    esac
fi

if [[ $# -gt 2 ]]; then
    usage
    exit 1
fi

if [[ $# -ge 1 ]]; then
    bootstrap_carla_dir "$1" "${2:-}"
    exit 0
fi

detected_dirs=()
for candidate_dir in \
    "$REPO_ROOT"/carla/CARLA_0.9.13 \
    "$REPO_ROOT"/carla/CARLA_0.9.15 \
    "$REPO_ROOT"/CARLA_0.9.13 \
    "$REPO_ROOT"/CARLA_0.9.15
do
    if is_carla_install_dir "$candidate_dir"; then
        detected_dirs+=("$candidate_dir")
    fi
done

if [[ ${#detected_dirs[@]} -eq 0 ]]; then
    die "No local CARLA installation found under the repository root (expected `carla/CARLA_0.9.13`, `carla/CARLA_0.9.15`, or the legacy root-level directories)."
fi

for detected_dir in "${detected_dirs[@]}"; do
    bootstrap_carla_dir "$detected_dir"
done
