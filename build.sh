#!/bin/bash
# Build script to copy Lua scripts to OpenTX SD card

# Target directory on SD card
TARGET_DIR="/media/forge/304E-B901/SCRIPTS/TOOLS"

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "========================================"
echo "  OpenTX Lua Script Build & Deploy"
echo "========================================"
echo ""

# Check if target directory exists
if [ ! -d "$TARGET_DIR" ]; then
    echo -e "${RED}ERROR: Target directory not found: $TARGET_DIR${NC}"
    echo "Is the SD card mounted?"
    exit 1
fi

# Find and copy all Lua files
echo ""
echo -e "${YELLOW}Copying Lua scripts...${NC}"

# Find all .lua files in current directory and subdirectories
LUA_FILES=$(find . -maxdepth 2 -name "*.lua" -type f | sort)

if [ -z "$LUA_FILES" ]; then
    echo -e "${RED}✗ No Lua files found${NC}"
    exit 1
fi

COPY_COUNT=0
for file in $LUA_FILES; do
    # Get just the filename (strip leading ./)
    filename=$(basename "$file")
    
    if cp "$file" "$TARGET_DIR/$filename"; then
        echo -e "${GREEN}✓ $filename copied${NC}"
        ((COPY_COUNT++))
    else
        echo -e "${RED}✗ Failed to copy $filename${NC}"
    fi
done

# Show summary
echo ""
echo "========================================"
echo -e "${GREEN}Build completed!${NC}"
echo "========================================"
echo ""
echo "Files deployed to: $TARGET_DIR"
echo "Total scripts copied: $COPY_COUNT"
echo ""
