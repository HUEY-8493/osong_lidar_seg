#!/usr/bin/env bash

echo -e "\e[1;34m--- CycloneDDS Configuration Verification ---\e[0m"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "CYCLONEDDS_URI:     $CYCLONEDDS_URI"

# Extract path from CYCLONEDDS_URI (remove file:// prefix if present)
xml_path="${CYCLONEDDS_URI#file://}"

if [ -n "$xml_path" ] && [ -f "$xml_path" ]; then
    echo -e "\e[1;32m[PASS] XML configuration file exists at: $xml_path\e[0m"
    echo -e "\e[1;36mXML content:\e[0m"
    cat "$xml_path"
else
    echo -e "\e[1;31m[FAIL] XML configuration file NOT found at: $xml_path\e[0m"
fi
