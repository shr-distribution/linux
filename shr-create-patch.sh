#!/bin/sh

cat <<EOF > shr.patch
All patches from shr kernel repository
rebased on top of openmoko kernel repository (om-2.6.39-stable)

https://gitorious.org/shr/linux/commits/shr-2.6.39-nodrm

$(git log --pretty=format:"%h: %s" gitorious/om-2.6.39-stable..)

$(git diff om-2.6.39-stable)
EOF
