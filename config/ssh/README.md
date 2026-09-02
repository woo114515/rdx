# SSH Access

The RDK X5 currently uses the `sunrise` account. The direct Ethernet address is assigned by DHCP and may change; verify it with `ip neigh show dev enp129s0` before connecting.

The public key in this directory identifies the current development workstation. Its private key is stored locally at `~/.ssh/rdx_ed25519` and must never be copied into this repository.

Connect with:

```bash
ssh -o IdentitiesOnly=yes \
  -i ~/.ssh/rdx_ed25519 \
  sunrise@10.42.0.72
```

Public-key fingerprint:

```text
SHA256:KssO9N++IOdNfdp/SOFp6rlSLhH9qH0jcspOECjPuPc
```

Each teammate should generate a separate key pair and install only their public key. Do not share private keys.
