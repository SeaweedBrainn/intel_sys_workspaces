# Remote Access to the Jetson

How to reach the Jetson (`autonomy-desktop`) from your laptop over SSH, so you can
build, run tests, and drive it (including from Claude Code) without sitting at the board.

## How it works

- **Network:** the Jetson is on our Tailscale tailnet at `100.91.100.16`. Your laptop must
  be signed in to the same tailnet (or have the Jetson shared with it). This works from
  any network, not just the lab Wi-Fi.
- **Login:** key-based SSH as the `autonomy` user. Each person has their own key; the
  public halves live in `~autonomy/.ssh/authorized_keys` on the Jetson.
- **Shortcut:** an entry in `~/.ssh/config` maps `jetson` to the IP, user, and key, so
  `ssh jetson` just works.

## Setup for a new user

### 1. Join the tailnet

Install Tailscale on your laptop and ask an admin to invite you to the tailnet (or share
the Jetson with you). Check with:

```bash
ping 100.91.100.16
```

### 2. Create a key (on your laptop)

```bash
ssh-keygen -t ed25519 -N "" -f ~/.ssh/jetson_key -C "<your-name>-to-jetson"
```

`-N ""` means no passphrase, so tools (and Claude Code) can connect non-interactively.
Anyone with this file can log in to the Jetson, so keep it on your own machine only.

### 3. Add the SSH config entry (on your laptop)

```bash
cat >> ~/.ssh/config <<'EOF'

Host jetson
  HostName 100.91.100.16
  User autonomy
  IdentityFile ~/.ssh/jetson_key
EOF
```

### 4. Get your public key authorized

Send `~/.ssh/jetson_key.pub` to someone who already has access. They run:

```bash
ssh jetson "echo '<your public key line>' >> ~/.ssh/authorized_keys"
```

Or, if you know the `autonomy` password, do it yourself from a regular terminal:

```bash
ssh-copy-id -i ~/.ssh/jetson_key.pub autonomy@100.91.100.16
```

### 5. Test it

```bash
ssh -o BatchMode=yes jetson 'echo ok; hostname'
```

`BatchMode=yes` fails instead of prompting, so if this prints `ok` you're set up for
password-free and scripted access.

## Everyday use

```bash
ssh jetson                                   # interactive shell
ssh jetson 'cd ~/intel_sys_workspaces && ./run_docker.sh'   # one-off command
scp results/foo.json jetson:~/intel_sys_workspaces/results/  # copy files
```

VS Code's Remote-SSH extension also picks up the `jetson` host.

### With Claude Code

Run Claude Code on your laptop. Once `ssh jetson` works, Claude can run commands, edit
files, and run tests on the Jetson by issuing `ssh jetson '...'` commands. The project
lives at `~/intel_sys_workspaces` on the Jetson.

## Revoking access

Each key line in `~autonomy/.ssh/authorized_keys` ends with a comment like
`<name>-to-jetson`. Delete that line to revoke that person.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ssh: connect to host ... timed out` | Not on the tailnet, or Tailscale is down on either machine. Check `tailscale status`. |
| `Permission denied (publickey,password)` with `-v` showing `Server accepts key` | Your private key has a passphrase and nothing can enter it. Use a key with no passphrase (step 2), or load it with `ssh-add --apple-use-keychain <key>` on macOS. |
| `Permission denied` and the key is never accepted | Your public key isn't in `authorized_keys`, or the config points to the wrong `IdentityFile`. |
| `ssh-copy-id` fails with `ssh_askpass: ... No such file` | You ran it somewhere that can't prompt for a password (e.g. Claude Code's `!` prompt). Run it in a normal terminal. |

## Scaling up (optional)

- **Tailscale SSH:** run `sudo tailscale up --ssh` on the Jetson to manage access through
  tailnet access rules instead of individual keys. Access ends when someone is removed
  from the tailnet.
- **Per-person accounts:** `sudo adduser <name>` on the Jetson if you need to track who
  did what or keep people from stepping on each other's work, instead of sharing `autonomy`.
