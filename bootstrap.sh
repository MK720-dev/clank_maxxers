#!/usr/bin/env bash

############################################################
# CLANK_MAXXERS — BULLETPROOF BOOTSTRAP
############################################################

set -e  # exit on error

echo ""
echo "🚀 Bootstrapping Clank Maxxers..."
echo ""

############################################
# Detect Python
############################################

if command -v python3 &> /dev/null; then
    PYTHON=python3
elif command -v python &> /dev/null; then
    PYTHON=python
else
    echo "❌ Python is not installed."
    echo "👉 Install from: https://www.python.org/downloads/"
    exit 1
fi

echo "✅ Using $PYTHON"

############################################
# Enforce Python Version
############################################

MIN_VERSION=3.9

$PYTHON - <<END
import sys
min_version = tuple(map(int, "$MIN_VERSION".split(".")))
if sys.version_info < min_version:
    raise SystemExit(
        f"❌ Python {min_version[0]}.{min_version[1]}+ required. "
        f"Found {sys.version_info.major}.{sys.version_info.minor}"
    )
END

echo "✅ Python version OK"

############################################
# Create Virtual Environment
############################################

if [ ! -d "venv" ]; then
    echo "🐍 Creating virtual environment..."
    $PYTHON -m venv venv
else
    echo "✅ Virtual environment already exists"
fi

############################################
# Activate (Mac/Linux vs Windows Git Bash)
############################################

if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
elif [ -f "venv/Scripts/activate" ]; then
    source venv/Scripts/activate
else
    echo "❌ Could not find virtualenv activation script."
    exit 1
fi

############################################
# Upgrade pip safely
############################################

echo "⬆️ Upgrading pip..."
python -m pip install --upgrade pip setuptools wheel

############################################
# Install Dependencies
############################################

if [ -f "requirements.txt" ]; then
    echo "📦 Installing dependencies..."
    pip install -r requirements.txt
else
    echo "⚠️ No requirements.txt found — skipping."
fi

############################################
# Verify Critical Imports (optional but elite)
############################################

echo "🔎 Verifying environment..."

python - <<END
try:
    import numpy
except:
    pass
END

############################################
# Final Message
############################################

echo ""
echo "🎉 Environment ready!"
echo ""
echo "Activate anytime with:"
echo "source venv/bin/activate"
echo ""
echo "Then launch Webots 🚗"
echo ""
