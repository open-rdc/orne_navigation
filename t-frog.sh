ypspur-coordinator -p icart-mini.param -d /dev/sensors/t_frog &
while true
do
isAlive=`ps -ef | grep " ypspur-coordinator " | \
grep -v grep | wc -l`
if [ $isAlive = 1 ]; then
: #Idling
else
echo "ypspur restart"
ypspur-coordinator -p icart-mini.param -d /dev/sensors/t_frog &
fi
sleep 1
done
