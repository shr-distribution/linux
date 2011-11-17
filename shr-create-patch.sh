#!/bin/sh

cat <<EOF > shr.patch
All patches from shr kernel repository
rebased on top latest stable patch (linux-stable tag v3.5 now)

https://gitorious.org/shr/linux/commits/shr-3.5-nodrm

$(git log --pretty=format:"%h: %s" v3.5..)

$(git diff v3.5)
EOF
