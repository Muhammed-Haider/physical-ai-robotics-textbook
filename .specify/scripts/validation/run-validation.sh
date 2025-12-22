#!/bin/bash
set -x

# This script simulates running validation agents on changed files.
# In a real scenario, this would invoke actual validation logic.

VALIDATION_OUTPUT_FILE="validation_output.txt"
VALIDATION_SUMMARY_FILE="validation_summary.txt"
VALIDATION_STATUS="SUCCESS"

echo "Running validation agents on changed files..." > "$VALIDATION_OUTPUT_FILE"
echo "--- Detailed Validation Output ---" >> "$VALIDATION_OUTPUT_FILE"

if [ -z "$@" ]; then
  echo "No files provided for validation." >> "$VALIDATION_OUTPUT_FILE"
  echo "Overall validation PASSED." >> "$VALIDATION_SUMMARY_FILE"
  echo "VALIDATION_STATUS=SUCCESS" >> "$VALIDATION_SUMMARY_FILE"
  exit 0
fi

validation_failed_count=0
total_files_validated=0

for file in "$@"; do
  total_files_validated=$((total_files_validated + 1))
  echo "Validating file: $file" >> "$VALIDATION_OUTPUT_FILE"
  # Simulate validation logic
  if [[ "$file" == *"fail"* ]]; then # Example: files containing "fail" in their name will fail validation
    echo "Validation FAILED for $file" >> "$VALIDATION_OUTPUT_FILE"
    validation_failed_count=$((validation_failed_count + 1))
    VALIDATION_STATUS="FAILURE"
  else
    echo "Validation PASSED for $file" >> "$VALIDATION_OUTPUT_FILE"
  fi
done

echo "--- Summary ---" > "$VALIDATION_SUMMARY_FILE"
echo "Total files validated: $total_files_validated" >> "$VALIDATION_SUMMARY_FILE"
echo "Files with failed validation: $validation_failed_count" >> "$VALIDATION_SUMMARY_FILE"

if [ "$validation_failed_count" -gt 0 ]; then
  echo "Overall validation FAILED." >> "$VALIDATION_SUMMARY_FILE"
  echo "VALIDATION_STATUS=FAILURE" >> "$VALIDATION_SUMMARY_FILE"
  exit 1
else
  echo "Overall validation PASSED." >> "$VALIDATION_SUMMARY_FILE"
  echo "VALIDATION_STATUS=SUCCESS" >> "$VALIDATION_SUMMARY_FILE"
  exit 0
fi