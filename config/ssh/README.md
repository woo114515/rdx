# SSH Access

The RDK X5 currently uses the `sunrise` account. Direct Ethernet and mobile hotspot addresses are assigned by DHCP and may change; verify the current address from the hotspot client list or with `ip neigh show dev <interface>` before connecting.

The public key in this directory identifies the current development workstation. Its private key is stored locally at `~/.ssh/rdx_ed25519` and must never be copied into this repository.

Connect with:

```bash
robot_ip=192.168.49.25  # Replace with the current DHCP address.
ssh -o IdentitiesOnly=yes \
  -i ~/.ssh/rdx_ed25519 \
  sunrise@"${robot_ip}"
```

Public-key fingerprint:

```text
SHA256:KssO9N++IOdNfdp/SOFp6rlSLhH9qH0jcspOECjPuPc
```

Each teammate should generate a separate key pair and install only their public key. Do not share private keys.

The board hostname is the generic `ubuntu`, so hostname discovery may be ambiguous when several
Ubuntu devices share a network. Prefer a DHCP reservation keyed by the board's Wi-Fi MAC; otherwise
confirm the address before connecting. When the lease changes, old SSH sessions can remain stuck
even while a new address works. Do not restart `NetworkManager` or `wpa_supplicant` while teammates
are connected. See [the 2026-09-03 incident record](../../docs/incident-2026-09-03-kernel-panic.md)
for the confirmed network-service risk and recovery boundary.
