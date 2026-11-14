#!/bin/sh
rm -rf docs
doxygen
git add docs
