#!/usr/bin/env bash

REMOTE_USER="mdp"
REMOTE_HOST="${RPI_HOST:-192.168.28.1}"
REMOTE_PASS="pi"
REMOTE_DIR="MDP"
REMOTE_CMD="cd ${REMOTE_DIR} && sudo bash -lc 'exec python3 task2.py'"

expect <<EOF
  set timeout -1

  trap {
    if {[info exists spawn_id]} {
      send \003
    }
  } SIGINT

  spawn ssh -tt -o StrictHostKeyChecking=no ${REMOTE_USER}@${REMOTE_HOST} "${REMOTE_CMD}"

  expect {
    -re "Are you sure you want to continue connecting.*" {
      send "yes\r"
      exp_continue
    }
    -re "(?i)password:" {
      send "${REMOTE_PASS}\r"
      exp_continue
    }
    eof
  }
EOF
