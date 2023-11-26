#!/bin/bash

function setup_xauth() {
  local XAUTH=/tmp/.docker.xauth

  if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ -n "$xauth_list" ]; then
      echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
      touch $XAUTH
    fi
    chmod a+r $XAUTH
  fi
}

setup_xauth
