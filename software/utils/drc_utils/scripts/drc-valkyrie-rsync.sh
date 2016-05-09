#!/bin/bash

read -p "WARNING: This will copy all files from ~/openhumanoids/oh-distro to ~/openhumanoids/oh-distro on link02. Okay? [yN]: " answer
case ${answer:0:1} in
	y|Y )
		rsync -av --exclude-from=$HOME/openhumanoids/oh-distro/software/utils/drc_utils/scripts/drc_valkyrie_rsync_excludes.txt $HOME/openhumanoids/oh-distro/ link02:$HOME/openhumanoids/oh-distro
	;;
	* )
		echo "Cancelled."
		exit
	;;
esac
