#!/bin/bash
# Attaches to the simulation session without nesting tmux (both the user and the mrs tmux use the C-a prefix, so a
# nested session gets no keys). Inside tmux, the current client is swapped for the simulation one, like tmux/start.sh
# does; the outer session keeps running (e.g. crash_hunt.sh), detach (C-a d) and run `tmux a` to get back to it.

if ! tmux -L mrs has-session -t simulation 2> /dev/null; then
  echo "no simulation session running" >&2
  exit 1
fi

if [ -z "$TMUX" ]; then
  exec tmux -L mrs a -t simulation
else
  tmux detach-client -E "tmux -L mrs a -t simulation"
fi
