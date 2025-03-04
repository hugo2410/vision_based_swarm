import os
import re

# Define the project name and description
PROJECT_NAME = "Vision Based Swarming"
PROJECT_DESCRIPTION = "A software package for autonomous drone swarming."

# Directory to scan for C++ source/header files
SOURCE_DIR = "src"

# Output docs.h file
OUTPUT_FILE = "docs.h"

# Regex pattern to extract class and function names
CLASS_PATTERN = re.compile(r"\bclass\s+(\w+)")
FUNCTION_PATTERN = re.compile(r"^\s*(?:\w+::)?(\w+)\s*\(.*\)\s*\{?", re.MULTILINE)

# Collect documented classes and functions
classes = set()
functions = set()

for root, _, files in os.walk(SOURCE_DIR):
    for file in files:
        if file.endswith((".h", ".cpp")):
            with open(os.path.join(root, file), "r", encoding="utf-8") as f:
                content = f.read()
                classes.update(CLASS_PATTERN.findall(content))
                functions.update(FUNCTION_PATTERN.findall(content))

# Generate the docs.h file content
docs_content = f"""/**
 * @file docs.h
 * @mainpage {PROJECT_NAME} Documentation
 *
 * @section intro Introduction
 * {PROJECT_DESCRIPTION}
 *
 * @section features Features
 * - Multi-agent coordination
 * - Real-time computer vision processing
 * - ROS2 integration
 *
 * @section classes Key Classes
 * {"".join([f" * - @ref {cls}\\n" for cls in sorted(classes)])}
 *
 * @section functions Key Functions
 * {"".join([f" * - {func}()\\n" for func in sorted(functions)])}
 *
 * @section installation Installation
 * ```
 * git clone https://github.com/hugo2410/vision_based_swarm
 * cd project
 * make install
 * ```
 *
 * @section usage Usage
 * ```
 * ./swarm_simulator --config config.yaml
 * ```
 *
 * @section contact Contact
 * For more details, visit [GitHub](https://github.com/hugo2410/vision_based_swarm) or contact hugobirch@hotmail.fr.
 */

#ifndef DOCS_H
#define DOCS_H
// This file is only for documentation purposes and does not contain code.
#endif // DOCS_H
"""

# Write the content to docs.h
with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
    f.write(docs_content)

print(f"Generated {OUTPUT_FILE} with {len(classes)} classes and {len(functions)} functions.")
