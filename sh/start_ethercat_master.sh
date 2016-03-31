#!/bin/bash


judge_strings=$(ifconfig | grep -A 1 "HWaddr")

inet6_strings=$(echo "${judge_strings}" | grep -B 1 "inet6")

ip_strings=$(echo "${inet6_strings}" | grep "HWaddr")

if [ -z "$ip_strings" ]
then
	echo "can not find EtherCat port, maybe already started, try to list slaves:"
	/opt/etherlab/bin/ethercat sla
	exit
fi

ip_address=${ip_strings#*HWaddr }
ip_address=${ip_address:0:17}
echo "ip address is: ${ip_address}"

sed "s/\*\*:\*\*:\*\*:\*\*:\*\*:\*\*/$ip_address/g" /usr/aris/resource/aris_control/ethercat.conf_backup > /etc/ethercat.conf
sed "s/\*\*:\*\*:\*\*:\*\*:\*\*:\*\*/$ip_address/g" /usr/aris/resource/aris_control/ethercat_backup > /etc/sysconfig/ethercat
cp /etc/ethercat.conf /opt/etherlab/etc/ethercat.conf
cp /etc/sysconfig/ethercat /opt/etherlab/etc/sysconfig/ethercat

/opt/etherlab/etc/init.d/ethercat start
/opt/etherlab/etc/init.d/ethercat restart
/opt/etherlab/etc/init.d/ethercat status

sleep 3

/opt/etherlab/bin/ethercat sla
