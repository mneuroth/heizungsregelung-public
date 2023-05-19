cd ~/Dokumente/projects/heizungsregelung
lxterminal --command="/bin/bash --init-file '~/Dokumente/projects/heizungsregelung/strompi2/start_watchdog.sh'"
netstat -a | grep 42424
while [ $? -eq 0 ]
do
    python -c "import time; print('waiting for available port 42424...'); time.sleep(5)"
    netstat -a | grep 42424
done
lxterminal --command="/bin/bash --init-file '~/Dokumente/projects/heizungsregelung/start_exporter.sh'"
#./start_chromium.sh &
./start_firefox.sh &
while true
do
   echo "============> starting heating control <=============="
   sudo python heizung.py -s -w -p
   echo "------------> heating control stoped ! <--------------"
   python -c "import time; time.sleep(15)"
done
