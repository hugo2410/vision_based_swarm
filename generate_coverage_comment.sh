#!/bin/bash
set -e

# Extract the coverage percentage from lcov output
COVERAGE=$(lcov --list coverage.info | grep 'Total:' | grep -o '[0-9.]\+%' | cut -d'%' -f1)

# Get the previous coverage from a stored value (if it exists)
if [ -f ".previous_coverage" ]; then
  PREVIOUS_COVERAGE=$(cat .previous_coverage)
else
  PREVIOUS_COVERAGE=$COVERAGE
fi

# Calculate the difference
DIFF=$(echo "$COVERAGE - $PREVIOUS_COVERAGE" | bc)

# Determine if coverage increased or decreased
if (( $(echo "$DIFF > 0" | bc -l) )); then
  CHANGE="increased by $DIFF%"
  EMOJI="🎉"
elif (( $(echo "$DIFF < 0" | bc -l) )); then
  CHANGE="decreased by $(echo "$DIFF * -1" | bc)%"
  EMOJI="🔻"
else
  CHANGE="unchanged"
  EMOJI="🔄"
fi

# Create markdown for the comment
cat > coverage-comment.md << EOF
## Code Coverage Report ${EMOJI}

Current coverage: **${COVERAGE}%**
Previous coverage: **${PREVIOUS_COVERAGE}%**
Coverage has ${CHANGE}

<details>
<summary>Click to see detailed coverage report</summary>

\`\`\`
$(lcov --list coverage.info)
\`\`\`

</details>
EOF

# Store the current coverage for future comparison
echo "$COVERAGE" > .previous_coverage
