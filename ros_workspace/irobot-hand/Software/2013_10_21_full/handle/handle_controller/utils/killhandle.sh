#! /bin/bash

HANDLEID=`ps ax | grep handle_ | grep -v grep | xargs echo | cut -d" " -s -f1`
if [ -n "$HANDLEID" ]
then
    kill $HANDLEID
else
    echo "WARNING: 'handle_' not found"
fi
