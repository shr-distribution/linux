#!/bin/sh

cat <<EOF > shr.patch
All patches from shr kernel repository
rebased on top latest stable patch (linux-stable tag v3.2.1 now)

https://gitorious.org/shr/linux/commits/shr-3.2-nodrm

$(git log --pretty=format:"%h: %s" v3.2.1..)

$(git diff v3.2.1)
EOF
