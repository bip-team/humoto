#!/bin/sh

cat $1 | grep -o "^humoto.* =\|^%.*"
