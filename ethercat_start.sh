#!/bin/sh
ip_strings=$(ifconfig | grep "eth")
ip_address=${ip_strings#*HWaddr }
#ip_address=${ip_address}
echo "ip address is:"
echo ${ip_address}

sed "s/\*\*:\*\*:\*\*:\*\*:\*\*:\*\*/$ip_address/g" /etc/ethercat.conf_backup > /etc/ethercat.conf
sed "s/\*\*:\*\*:\*\*:\*\*:\*\*:\*\*/$ip_address/g" /etc/sysconfig/ethercat_backup > /etc/sysconfig/ethercat
cp /etc/ethercat.conf /opt/etherlab/etc/ethercat.conf
cp /etc/sysconfig/ethercat /opt/etherlab/etc/sysconfig/ethercat

/opt/etherlab/etc/init.d/ethercat start
/opt/etherlab/etc/init.d/ethercat restart
/opt/etherlab/etc/init.d/ethercat status
/opt/etherlab/bin/ethercat sla
