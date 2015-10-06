PARAM=${1:-icart-mini.param}
ypspur-coordinator -p $PARAM -d /dev/sensors/icart-mini &
while true
do
isAlive=`ps -ef | grep " ypspur-coordinator " | \
grep -v grep | wc -l`
if [ $isAlive = 1 ]; then
: #pass
else
echo "ypspur restart"
ypspur-coordinator -p $PARAM -d /dev/sensors/icart-mini &
fi
sleep 1
done
