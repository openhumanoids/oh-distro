#!/bin/bash
all_brainstems="cyberdisc-1.local,cyberdisc-2.local,cyberdisc-3.local,cyberdisc-4.local"
brainstems=
all=1
cmd=
user='robonaut'
user4='ubuntu'
usage() { echo "Usage: $0 [-u <user>] [-b <comma delimeted list of computers>] <shutdown|reboot>" 1>&2; exit 1; }

while getopts b:u:h opt; do
	case $opt in
		b)
			all=0
			brainstems=$OPTARG
			echo "brainstems: " $brainstems
			;;
		u)
			user=$OPTARG
			echo "user: " $user
			;;
		h)
			usage
			;;
		*)
			usage
			;;
	esac
done

shift $((OPTIND - 1))
cmd="$*"

if [ ${all} -eq 1 ]; then
	brainstems=$all_brainstems
fi

IFS=',' read -ra systems <<< "$brainstems"
for sys in "${systems[@]}"; do
	if [ "${sys}" == "cyberdisc-4.local" ]; then
		ssh_user=${user4}
	else
		ssh_user=${user}
	fi
	if [ "${cmd}" == "shutdown" ]; then
		echo "Shutting down ${sys}"
		ssh -t ${ssh_user}@${sys} "sudo shutdown -P now 'system is performing a commanded shutdown'"
	elif [ "${cmd}" == "reboot" ]; then
		echo "Rebooting ${sys}"
		ssh -t ${ssh_user}@${sys} "sudo shutdown -r now 'system is performing a commanded reboot'"
	else
		echo "Missing shutdown/reboot argument, exiting"
		exit 1;
	fi
done
