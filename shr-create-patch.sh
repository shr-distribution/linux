#!/bin/sh

cat <<EOF > shr.patch
All patches from shr kernel repository
rebased on top of openmoko kernel repository (om-gta02/2.6.39/om-stable)

http://git.shr-project.org/git/?p=linux.git;a=shortlog;h=refs/heads/om-gta02/2.6.39/master

$(git log --pretty=format:"%h: %s" shr/om-gta02/2.6.39/om-stable..)

$(git diff shr/om-gta02/2.6.39/om-stable)
EOF
