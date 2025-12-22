#!/bin/bash

# Unit tests for run-validation.sh

# Get the directory where the test script is located
TEST_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Construct the absolute path to the script under test
SCRIPT_TO_TEST_ABS="$TEST_SCRIPT_DIR/../../.specify/scripts/validation/run-validation.sh"

TEST_TEMP_DIR="test_temp"

# Create and change to a temporary directory for test outputs
mkdir -p "$TEST_TEMP_DIR"
cd "$TEST_TEMP_DIR" || exit 1

echo "Running tests for $SCRIPT_TO_TEST_ABS"

# Test Case 1: No files provided, should pass
echo "--- Test Case 1: No files ---"
"$SCRIPT_TO_TEST_ABS" > /dev/null 2>&1 # Suppress output for clean tests
EXIT_CODE=$?
SUMMARY=$(cat validation_summary.txt)
echo "Exit Code: $EXIT_CODE"
echo "Summary: $SUMMARY"
if [[ "$EXIT_CODE" -eq 0 && "$SUMMARY" == *"No files provided for validation."* && "$SUMMARY" == *"VALIDATION_STATUS=SUCCESS"* ]]; then
  echo "Test Case 1 PASSED"
else
  echo "Test Case 1 FAILED"
  exit 1
fi
rm validation_summary.txt validation_output.txt

# Test Case 2: All files pass, should pass
echo "--- Test Case 2: All files pass ---"
touch file1.md file2.js
"$SCRIPT_TO_TEST_ABS" file1.md file2.js > /dev/null 2>&1 # Suppress output for clean tests
EXIT_CODE=$?
SUMMARY=$(cat validation_summary.txt)
echo "Exit Code: $EXIT_CODE"
echo "Summary: $SUMMARY"
if [[ "$EXIT_CODE" -eq 0 && "$SUMMARY" == *"Overall validation PASSED."* && "$SUMMARY" == *"VALIDATION_STATUS=SUCCESS"* ]]; then
  echo "Test Case 2 PASSED"
else
  echo "Test Case 2 FAILED"
  exit 1
fi
rm file1.md file2.js validation_summary.txt validation_output.txt

# Test Case 3: One file fails, should fail
echo "--- Test Case 3: One file fails ---"
touch file1.md fail_file.js
"$SCRIPT_TO_TEST_ABS" file1.md fail_file.js > /dev/null 2>&1 # Suppress output for clean tests
EXIT_CODE=$?
SUMMARY=$(cat validation_summary.txt)
echo "Exit Code: $EXIT_CODE"
echo "Summary: $SUMMARY"
if [[ "$EXIT_CODE" -ne 0 && "$SUMMARY" == *"Overall validation FAILED."* && "$SUMMARY" == *"Files with failed validation: 1"* && "$SUMMARY" == *"VALIDATION_STATUS=FAILURE"* ]]; then
  echo "Test Case 3 PASSED"
else
  echo "Test Case 3 FAILED"
  exit 1
fi
rm file1.md fail_file.js validation_summary.txt validation_output.txt

echo "All tests completed."
cd ..
rm -r "$TEST_TEMP_DIR"
