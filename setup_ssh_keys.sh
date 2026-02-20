#!/bin/bash
# One-time setup: copy your SSH key to Nano and Orin so start_system.sh can start them without a password.
# You will be prompted for the password twice (once per host). Use: jetbot

set -e
NANO_IP="10.13.68.184"
ORIN_IP="10.13.68.159"

KEY="${HOME}/.ssh/id_ed25519.pub"
[ -f "$KEY" ] || KEY="${HOME}/.ssh/id_rsa.pub"
if [ ! -f "$KEY" ]; then
    echo "No SSH public key found at ~/.ssh/id_ed25519.pub or ~/.ssh/id_rsa.pub"
    echo "Create one with: ssh-keygen -t ed25519 -N '' -f ~/.ssh/id_ed25519"
    exit 1
fi

echo "Using key: $KEY"
echo ""
echo "You will be asked for the password twice. Password: jetbot"
echo ""

echo "1. Copying key to Nano (jetbot@${NANO_IP})..."
ssh-copy-id -i "$KEY" jetbot@${NANO_IP}

echo ""
echo "2. Copying key to Orin (jetbot@${ORIN_IP})..."
ssh-copy-id -i "$KEY" jetbot@${ORIN_IP}

echo ""
echo "✅ Done. You can now run ./start_system.sh to start Controller, Nano, and Orin without entering passwords."
