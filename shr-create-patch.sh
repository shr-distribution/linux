#!/bin/sh

cat <<EOF > shr.patch
All patches from shr kernel repository
rebased on top of linux/master kernel repository

https://gitorious.org/shr/linux/commits/shr-master

$(git log --pretty=format:"%h: %s" linux/master..)

$(git diff linux/master)
EOF
