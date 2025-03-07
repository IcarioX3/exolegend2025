#!/bin/bash

[ -d $1/src ] && cp -r $1/src .
[ -d $1/inc ] && cp -r $1/inc .

echo "Do you want to auto push the new changes ? y/N"
read response

if [[ $response == "y" ]]; then
	git add .
	git commit -m "commit from $USER at `date`"
	git push
fi

echo "Changes saved!"
