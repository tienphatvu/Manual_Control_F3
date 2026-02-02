#!/usr/bin/env bash

REMOTE="root@172.16.0.91"

FILES=(
/mnt/rcd/dump/dump_rcd_9190_20260202_110457~20260202_110512.hiklog

/mnt/rcd/dump/dump_rcd_9190_20260202_110444~20260202_110457.hiklog

/mnt/rcd/dump/dump_rcd_9190_20260202_110340~20260202_110445.hiklog

/mnt/rcd/dump/dump_rcd_9190_20260202_110235~20260202_110333.hiklog

/mnt/rcd/data/rcd_9190_20260202_110109~20260202_111004.hiklog

/mnt/evt/data/evt_9190_20260202_085258.hiklog
)

CMD="sz -b"
for f in "${FILES[@]}"; do
  CMD+=" $(printf %q "$f")"
done

echo "[*] SSH to $REMOTE"
echo "[*] When asked, enter password: Hik@1234"
echo "[*] ZMODEM download will start automatically"

ssh -t "$REMOTE" "$CMD"