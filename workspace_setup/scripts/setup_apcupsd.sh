#!/usr/bin/env bash
set -euo pipefail

APCUPSD_CONF="/etc/apcupsd/apcupsd.conf"
APCUPSD_DEFAULT="/etc/default/apcupsd"
APCUPSD_CONF_BACKUP="${APCUPSD_CONF}.workspace_setup.bak"
APCUPSD_DEFAULT_BACKUP="${APCUPSD_DEFAULT}.workspace_setup.bak"
SUDO=()

fail() {
  echo "[ERROR] $*" >&2
  exit 1
}

warn() {
  echo "[WARN] $*"
}

communication_unverified() {
  echo "APCUPSD installation/configuration completed."
  echo "UPS communication could not be verified."
  echo "Check the UPS USB cable and power, then run: apcaccess status"
}

service_diagnostics() {
  echo "[INFO] apcupsd service diagnostics:"
  "${SUDO[@]}" systemctl status apcupsd --no-pager -l || true
  if command -v journalctl >/dev/null 2>&1; then
    echo "[INFO] Recent apcupsd journal entries:"
    "${SUDO[@]}" journalctl -u apcupsd -n 50 --no-pager || true
  fi
}

backup_once() {
  local source_file="$1"
  local backup_file="$2"

  [[ -f "$source_file" ]] || fail "Required configuration file not found: ${source_file}"

  if "${SUDO[@]}" test -e "$backup_file"; then
    echo "[INFO] Keeping existing initial backup: ${backup_file}"
  else
    "${SUDO[@]}" cp -p "$source_file" "$backup_file" \
      || fail "Could not create backup: ${backup_file}"
    echo "[OK] Created initial backup: ${backup_file}"
  fi
}

configure_apcupsd_conf() {
  local temporary_file
  temporary_file="$(mktemp)"

  awk '
    function replace_or_disable(name, value,    line) {
      line = $0
      if (line ~ "^[[:space:]]*" name "([[:space:]]|$)") {
        count[name]++
        if (count[name] == 1) {
          print value
        } else {
          print "# workspace_setup: disabled duplicate " line
        }
        return 1
      }
      return 0
    }
    {
      if (replace_or_disable("UPSCABLE", "UPSCABLE usb")) next
      if (replace_or_disable("UPSTYPE", "UPSTYPE usb")) next
      if (replace_or_disable("DEVICE", "DEVICE")) next
      print
    }
    END {
      if (!count["UPSCABLE"]) print "UPSCABLE usb"
      if (!count["UPSTYPE"]) print "UPSTYPE usb"
      if (!count["DEVICE"]) print "DEVICE"
    }
  ' "$APCUPSD_CONF" > "$temporary_file" \
    || { rm -f "$temporary_file"; fail "Could not prepare ${APCUPSD_CONF}."; }

  "${SUDO[@]}" tee "$APCUPSD_CONF" >/dev/null < "$temporary_file" \
    || { rm -f "$temporary_file"; fail "Could not update ${APCUPSD_CONF}."; }
  rm -f "$temporary_file"
}

configure_apcupsd_default() {
  local temporary_file
  temporary_file="$(mktemp)"

  awk '
    /^[[:space:]]*ISCONFIGURED([[:space:]]|$)/ {
      count++
      if (count == 1) {
        print "ISCONFIGURED=yes"
      } else {
        print "# workspace_setup: disabled duplicate " $0
      }
      next
    }
    { print }
    END {
      if (!count) print "ISCONFIGURED=yes"
    }
  ' "$APCUPSD_DEFAULT" > "$temporary_file" \
    || { rm -f "$temporary_file"; fail "Could not prepare ${APCUPSD_DEFAULT}."; }

  "${SUDO[@]}" tee "$APCUPSD_DEFAULT" >/dev/null < "$temporary_file" \
    || { rm -f "$temporary_file"; fail "Could not update ${APCUPSD_DEFAULT}."; }
  rm -f "$temporary_file"
}

verify_configuration() {
  local config_counts default_count

  config_counts="$(awk '
    /^[[:space:]]*UPSCABLE[[:space:]]+usb[[:space:]]*$/ { cable++ }
    /^[[:space:]]*UPSTYPE[[:space:]]+usb[[:space:]]*$/ { type++ }
    /^[[:space:]]*DEVICE[[:space:]]*$/ { device++ }
    END { print cable + 0, type + 0, device + 0 }
  ' "$APCUPSD_CONF")" || fail "Could not verify ${APCUPSD_CONF}."
  [[ "$config_counts" == "1 1 1" ]] \
    || fail "${APCUPSD_CONF} does not contain exactly one USB UPSCABLE, UPSTYPE, and empty DEVICE setting."

  default_count="$(awk '/^[[:space:]]*ISCONFIGURED=yes[[:space:]]*$/ { count++ } END { print count + 0 }' "$APCUPSD_DEFAULT")" \
    || fail "Could not verify ${APCUPSD_DEFAULT}."
  [[ "$default_count" == "1" ]] \
    || fail "${APCUPSD_DEFAULT} does not contain exactly one ISCONFIGURED=yes setting."
}

if [[ "$EUID" -ne 0 ]]; then
  command -v sudo >/dev/null 2>&1 \
    || fail "sudo is required to install and configure apcupsd."
  SUDO=(sudo)
fi

if command -v apcaccess >/dev/null 2>&1; then
  echo "[INFO] apcupsd is already installed."
else
  echo "[1/5] Installing apcupsd..."
  "${SUDO[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y apcupsd \
    || fail "Failed to install apcupsd."
  command -v apcaccess >/dev/null 2>&1 \
    || fail "apcupsd installation completed, but apcaccess was not found."
fi

echo "[2/5] Backing up existing apcupsd configuration..."
backup_once "$APCUPSD_CONF" "$APCUPSD_CONF_BACKUP"
backup_once "$APCUPSD_DEFAULT" "$APCUPSD_DEFAULT_BACKUP"

echo "[3/5] Configuring apcupsd for USB UPS discovery..."
configure_apcupsd_conf
configure_apcupsd_default
verify_configuration
echo "[OK] Set UPSCABLE usb, UPSTYPE usb, DEVICE, and ISCONFIGURED=yes."

echo "[4/5] Enabling and restarting apcupsd..."
"${SUDO[@]}" systemctl enable apcupsd \
  || { service_diagnostics; fail "Could not enable apcupsd."; }
"${SUDO[@]}" systemctl restart apcupsd \
  || { service_diagnostics; fail "Could not restart apcupsd."; }

echo "[INFO] apcupsd enable state:"
if ! enabled_state="$("${SUDO[@]}" systemctl is-enabled apcupsd 2>&1)"; then
  printf '%s\n' "$enabled_state"
  service_diagnostics
  fail "apcupsd is not enabled."
fi
printf '%s\n' "$enabled_state"
if ! "${SUDO[@]}" systemctl is-enabled --quiet apcupsd; then
  service_diagnostics
  fail "apcupsd is not enabled."
fi

echo "[INFO] apcupsd active state:"
if ! active_state="$("${SUDO[@]}" systemctl is-active apcupsd 2>&1)"; then
  printf '%s\n' "$active_state"
  service_diagnostics
  fail "apcupsd is not active."
fi
printf '%s\n' "$active_state"
if ! "${SUDO[@]}" systemctl is-active --quiet apcupsd; then
  service_diagnostics
  fail "apcupsd is not active."
fi

echo "[5/5] Checking for an APC/Schneider USB device..."
if command -v lsusb >/dev/null 2>&1; then
  if lsusb | grep -Ei 'APC|American Power Conversion|Schneider'; then
    echo "[OK] Matching APC/Schneider USB device detected."
  else
    warn "No APC/Schneider USB device was found by lsusb. The UPS may be disconnected."
  fi
else
  warn "lsusb is not installed; skipping USB device listing."
fi

echo "[INFO] Attempting UPS communication check (timeout: 10 seconds)..."
if apcaccess_output="$(timeout 10 apcaccess status 2>&1)"; then
  printf '%s\n' "$apcaccess_output"
  if grep -Eq '^(STATUS|CABLE|DRIVER|BCHARGE|TIMELEFT|LINEV)[[:space:]]*:' <<< "$apcaccess_output"; then
    echo "[DONE] APCUPSD installation/configuration and UPS communication verified."
  else
    warn "apcaccess returned successfully but did not report recognizable UPS status fields."
    communication_unverified
  fi
else
  communication_status=$?
  printf '%s\n' "$apcaccess_output"
  warn "apcaccess status could not verify UPS communication (exit code: ${communication_status})."
  communication_unverified
fi
