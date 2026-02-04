#!/usr/bin/env bash
set -e

echo "🚀 Bootstrapping clank_maxxers..."

#####################################
# Check Python
#####################################

if ! command -v python3 &> /dev/null
then
    echo "❌ Python3 is required but not installed."
    exit
fi

#####################################
# Create venv ONLY if missing
#####################################

if [ ! -d "venv" ]; then
    echo "🐍 Creating virtual environment..."
    python3 -m venv venv
else
    echo "✅ Virtual environment already exists."
fi

#####################################
# Activate
#####################################

source ./venv/Scripts/activate

#####################################
# Upgrade pip
#####################################

python.exe -m pip install --upgrade pip

#####################################
# Install dependencies
#####################################

if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
fi

echo ""
echo "✅ Environment ready."
echo ""
echo "Activate anytime with:"
echo "source ./venv/Scripts/activate"
