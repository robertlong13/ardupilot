#!/bin/sh
#
# A script suitable for use as a git pre-push hook for devs that have some write
# acces to main ArduPilot repos who typically intend to push to their own forks.
# Those of us who use GUI tools can more easily push to the wrong remote with a
# careless click or keypress.
#
# To override this check temporarily, set the following environment variable:
#    IREALLYMEANIT=1 git push -u origin <branch>

if [ "$IREALLYMEANIT" = "1" ]; then
    exit 0
fi

remote_name="$1"
remote_url=$(git remote get-url "$remote_name" 2>/dev/null)

if echo "$remote_url" | grep -Ei 'github\.com[:/]+ardupilot/'; then
    echo "Blocked push to ArduPilot GitHub repo: $remote_url" >&2
    echo "Set IREALLYMEANIT=1 to override" >&2
    exit 1
fi

exit 0
