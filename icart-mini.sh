ypspur-coordinator -p icart-mini.param -d /dev/sensors/icart-mini &
while true
do
isAlive=`ps -ef | grep " ypspur-coordinator " | \
grep -v grep | wc -l`
if [ $isAlive = 1 ]; then
: #pass
else
echo "ypspur restart"
ypspur-coordinator -p icart-mini.param -d /dev/sensors/icart-mini &
fi
sleep 1
done
