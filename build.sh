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

# Create backup directory with timestamp
BACKUP_DIR="./backups/$(date +%Y%m%d_%H%M%S)"
echo -e "${YELLOW}Creating backup: $BACKUP_DIR${NC}"
mkdir -p "$BACKUP_DIR"

# Backup existing files if they exist
if [ -f "$TARGET_DIR/example-sim-game.lua" ]; then
    cp "$TARGET_DIR"/*.lua "$BACKUP_DIR/" 2>/dev/null
    echo -e "${GREEN}✓ Backup created${NC}"
fi

# Copy main game script
echo ""
echo -e "${YELLOW}Copying main game script...${NC}"
if cp example-sim-game.lua "$TARGET_DIR/"; then
    echo -e "${GREEN}✓ example-sim-game.lua copied${NC}"
else
    echo -e "${RED}✗ Failed to copy example-sim-game.lua${NC}"
    exit 1
fi

# Copy test scripts
echo ""
echo -e "${YELLOW}Copying test scripts...${NC}"
if cp test/test-require.lua "$TARGET_DIR/"; then
    echo -e "${GREEN}✓ test-require.lua copied${NC}"
else
    echo -e "${RED}✗ Failed to copy test-require.lua${NC}"
fi

if cp test/utils.lua "$TARGET_DIR/"; then
    echo -e "${GREEN}✓ utils.lua copied${NC}"
else
    echo -e "${RED}✗ Failed to copy utils.lua${NC}"
fi

# Show summary
echo ""
echo "========================================"
echo -e "${GREEN}Build completed!${NC}"
echo "========================================"
echo ""
echo "Files deployed to: $TARGET_DIR"
echo ""
echo "Scripts available in OpenTX:"
echo "  • example-sim-game.lua (FPV Simulator)"
echo "  • test-require.lua (Module Test)"
echo ""
echo "Run 'test-require' in OpenTX to verify"
echo "if require() works on your hardware."
echo ""
